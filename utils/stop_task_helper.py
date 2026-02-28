#!/usr/bin/env python3
"""
Unified stop helper for ego/track/perch modes.

The shell wrappers in scripts/ only need to:
1) source workspace environment
2) call this script with mode + tmp dir
"""

import argparse
import math
import os
import subprocess
import time
from typing import Iterable, List, Optional, Tuple

import rospy
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray


def run_cmd(cmd: List[str]) -> bool:
    """Run a command quietly; return True if exit code is 0."""
    result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return result.returncode == 0


def kill_pid_file(pid_file: str, label: str) -> None:
    if not os.path.isfile(pid_file):
        return

    pid_str = ""
    try:
        with open(pid_file, "r", encoding="utf-8") as f:
            pid_str = f.read().strip()
    except OSError:
        pass

    if pid_str.isdigit():
        pid = int(pid_str)
        try:
            os.kill(pid, 15)
            rospy.loginfo("  Terminating %s (PID: %d)", label, pid)
            time.sleep(0.2)
            os.kill(pid, 9)
        except OSError:
            pass

    try:
        os.remove(pid_file)
    except OSError:
        pass


def get_first_odom(topics: Iterable[str], timeout_sec: float = 1.0) -> Optional[Odometry]:
    for topic in topics:
        try:
            msg = rospy.wait_for_message(topic, Odometry, timeout=timeout_sec)
            rospy.loginfo("  Position source: %s", topic)
            return msg
        except rospy.ROSException:
            continue
    return None


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm > 1e-12:
        x /= norm
        y /= norm
        z /= norm
        w /= norm
    else:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def save_position_snapshot(tmp_dir: str, odom_topics: Iterable[str]) -> Tuple[float, float, float, float]:
    os.makedirs(tmp_dir, exist_ok=True)
    out_file = os.path.join(tmp_dir, "drone_position.tmp")

    x, y, z, yaw = 0.0, 0.0, 2.0, 0.0
    odom_msg = get_first_odom(odom_topics)
    if odom_msg is not None:
        x = float(odom_msg.pose.pose.position.x)
        y = float(odom_msg.pose.pose.position.y)
        z = float(odom_msg.pose.pose.position.z)
        q = odom_msg.pose.pose.orientation
        yaw = quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
    else:
        rospy.logwarn("  Unable to get odom, using default position.")

    orient_z = math.sin(yaw / 2.0)
    orient_w = math.cos(yaw / 2.0)

    content = (
        f"# Generation time: {time.ctime()}\n"
        "# Format: parameter_name=value\n\n"
        f"INIT_X={x}\n"
        f"INIT_Y={y}\n"
        f"INIT_Z={z}\n"
        f"INIT_YAW={yaw}\n"
        f"ORIENT_Z={orient_z}\n"
        f"ORIENT_W={orient_w}\n"
    )
    with open(out_file, "w", encoding="utf-8") as f:
        f.write(content)

    rospy.loginfo(
        "  Position snapshot saved: x=%.3f y=%.3f z=%.3f yaw=%.3f -> %s",
        x,
        y,
        z,
        yaw,
        out_file,
    )
    return x, y, z, yaw


def kill_nodes(nodes: Iterable[str]) -> None:
    for node in nodes:
        ok = run_cmd(["rosnode", "kill", node])
        if ok:
            rospy.loginfo("  Killed node: %s", node)


def pkill_patterns(patterns: Iterable[str]) -> None:
    for pattern in patterns:
        run_cmd(["pkill", "-f", pattern])


def publish_marker_delete(topic: str) -> None:
    pub = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
    rospy.sleep(0.05)
    msg = Marker()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.action = Marker.DELETEALL
    pub.publish(msg)


def publish_marker_array_delete(topic: str) -> None:
    pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.05)
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
    marker.action = Marker.DELETEALL
    arr = MarkerArray()
    arr.markers = [marker]
    pub.publish(arr)


def publish_empty_path(topic: str) -> None:
    pub = rospy.Publisher(topic, Path, queue_size=1, latch=True)
    rospy.sleep(0.05)
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "world"
    path.poses = []
    pub.publish(path)


def disable_ego_planning(service_name: str) -> None:
    ok = run_cmd(["rosservice", "call", service_name, "data: false"])
    if ok:
        rospy.loginfo("  Planning disabled by service: %s", service_name)
    else:
        rospy.logwarn("  Failed to call planning disable service: %s", service_name)


def handle_ego(tmp_dir: str) -> None:
    rospy.loginfo("=== Stopping Ego Related Nodes and Clearing Topics ===")
    save_position_snapshot(tmp_dir, ["/drone_0_visual_slam/odom", "/visual_slam/odom"])

    disable_ego_planning("/drone_0_ego_planner_node/planning/enable")

    kill_nodes(
        [
            "/drone_0_quadrotor_simulator_so3",
            "/drone_0_so3_control",
            "/drone_0_pcl_render_node",
            "/drone_0_odom_visualization",
            "/waypoint_generator",
            "/quadrotor_simulator_so3",
            "/so3_control",
            "/pcl_render_node",
            "/traj_server",
            "/drone_0_traj_server",
            "/odom_visualization",
        ]
    )
    pkill_patterns(
        [
            "rosrun.*traj_server$",
            "rosrun.*odom_visualization$",
            "rosrun.*quadrotor_simulator_so3$",
            "rosrun.*so3_control$",
            "rosrun.*pcl_render_node$",
        ]
    )

    publish_marker_delete("/drone_0_ego_planner_node/optimal_list")
    publish_marker_delete("/drone_0_ego_planner_node/goal_point")
    rospy.loginfo("=== EGO Stop Completed ===")


def handle_track(tmp_dir: str) -> None:
    rospy.loginfo("=== Stopping Tracker Related Nodes and Clearing Topics ===")
    save_position_snapshot(tmp_dir, ["/ekf/ekf_odom", "/odom"])

    kill_pid_file(os.path.join(tmp_dir, "track_real.pid"), "planning real_external.launch")
    pkill_patterns(["planning real_external.launch"])

    kill_nodes(
        [
            "/track/manager",
            "/track/mapping",
            "/track/mapping_vis",
            "/track/target_ekf_node",
            "/track/planning",
            "/track/traj_server",
        ]
    )

    publish_empty_path("/track/planning/traj")
    publish_marker_delete("/track/planning/visible_region")
    publish_marker_array_delete("/track/odom_visualization/fov_visual")
    publish_empty_path("/track/planning/astar")
    publish_empty_path("/track/planning/traj")
    rospy.loginfo("=== TRACK Stop Completed ===")


def handle_perch(tmp_dir: str) -> None:
    rospy.loginfo("=== Stopping Perching Related Nodes and Saving Position Information ===")
    save_position_snapshot(tmp_dir, ["/perch/planning/odom", "/odom"])

    kill_nodes(["/perch/odom_visualization", "/perch/odom_visualization_plate", "/perch/manager", "/perch/planning"])

    publish_marker_delete("/perch/odom_visualization_plate/polygon")
    publish_empty_path("/perch/planning/traj")
    publish_marker_array_delete("/perch/odom_visualization/fov_visual")
    rospy.loginfo("=== PERCH Stop Completed ===")


def main() -> None:
    parser = argparse.ArgumentParser(description="Unified stop helper")
    parser.add_argument("--mode", choices=["ego", "track", "perch"], required=True)
    parser.add_argument("--tmp-dir", required=True)
    args = parser.parse_args()

    rospy.init_node(f"stop_task_helper_{args.mode}", anonymous=True)

    if args.mode == "ego":
        handle_ego(args.tmp_dir)
    elif args.mode == "track":
        handle_track(args.tmp_dir)
    else:
        handle_perch(args.tmp_dir)


if __name__ == "__main__":
    main()
