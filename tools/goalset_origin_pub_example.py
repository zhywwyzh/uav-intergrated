#!/usr/bin/env python3
"""
Atomic publisher example:
1) Validate both publishing channels first.
2) If all checks pass, publish /task_id=1 first.
3) Then publish /goal_with_id_from_station_origin (GoalSet).
If any check fails, publish neither topic.
"""

import argparse

import rospy
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import GoalSet
from std_msgs.msg import Int32


TASK_TOPIC = "/task_id"
GOAL_ORIGIN_TOPIC = "/goal_with_id_from_station_origin"
TASK_MSG_TYPE = "std_msgs/Int32"
GOAL_MSG_TYPE = "quadrotor_msgs/GoalSet"


def check_topic_type_compatible(topic_name, expected_type):
    """Return True if topic is unpublished or has expected type."""
    try:
        topic_map = dict(rospy.get_published_topics())
    except rospy.ROSException as exc:
        rospy.logerr("Failed to query published topics: %s", exc)
        return False

    current_type = topic_map.get(topic_name)
    if current_type is None:
        return True
    if current_type == expected_type:
        return True

    rospy.logerr(
        "Topic %s type mismatch: current=%s, expected=%s",
        topic_name,
        current_type,
        expected_type,
    )
    return False


def build_goal_msg(x, y, z, yaw):
    """Build and validate a GoalSet message."""
    pose_msg = GoalSet()
    pose_msg.to_drone_ids = [0]

    pt = Point()
    pt.x = float(x)
    pt.y = float(y)
    pt.z = float(z)
    pose_msg.goal = [pt]
    pose_msg.yaw = [float(yaw)]
    pose_msg.look_forward = False
    pose_msg.goal_to_follower = False

    # Structure check: must be GoalSet and contain required fields.
    if not isinstance(pose_msg, GoalSet):
        return None
    if len(pose_msg.to_drone_ids) == 0 or len(pose_msg.goal) == 0 or len(pose_msg.yaw) == 0:
        return None
    return pose_msg


def main():
    parser = argparse.ArgumentParser(
        description="Publish /task_id=1 and /goal_with_id_from_station_origin atomically"
    )
    parser.add_argument("--x", type=float, default=0.0, help="target x")
    parser.add_argument("--y", type=float, default=0.0, help="target y")
    parser.add_argument("--z", type=float, default=1.0, help="target z")
    parser.add_argument("--yaw", type=float, default=0.0, help="target yaw (rad)")
    parser.add_argument("--task-id", type=int, default=1, help="task id to publish first")
    parser.add_argument(
        "--after-task-delay",
        type=float,
        default=1.0,
        help="seconds to wait after publishing /task_id and before publishing GoalSet",
    )
    args = parser.parse_args()

    rospy.init_node("goalset_origin_pub_example", anonymous=True)
    task_pub = rospy.Publisher(TASK_TOPIC, Int32, queue_size=10)
    goal_pub = rospy.Publisher(GOAL_ORIGIN_TOPIC, GoalSet, queue_size=10)

    # Wait briefly for publisher registration.
    rospy.sleep(0.5)

    goal_msg = build_goal_msg(args.x, args.y, args.z, args.yaw)
    if goal_msg is None:
        rospy.logerr("GoalSet content check failed. Abort: publish neither topic.")
        return

    task_ok = check_topic_type_compatible(TASK_TOPIC, TASK_MSG_TYPE)
    goal_ok = check_topic_type_compatible(GOAL_ORIGIN_TOPIC, GOAL_MSG_TYPE)
    if not (task_ok and goal_ok):
        rospy.logerr("Pre-check failed. Abort: publish neither topic.")
        return

    # Atomic order: publish task_id first, then goal.
    task_msg = Int32()
    task_msg.data = int(args.task_id)
    task_pub.publish(task_msg)
    rospy.loginfo("Published %s: %d", TASK_TOPIC, task_msg.data)

    rospy.sleep(max(0.0, float(args.after_task_delay)))
    goal_pub.publish(goal_msg)

    rospy.loginfo(
        "Published GoalSet to %s: "
        "drone_ids=%s, goal=(%.3f, %.3f, %.3f), yaw=%.3f, look_forward=%s, goal_to_follower=%s",
        GOAL_ORIGIN_TOPIC,
        list(goal_msg.to_drone_ids),
        goal_msg.goal[0].x,
        goal_msg.goal[0].y,
        goal_msg.goal[0].z,
        goal_msg.yaw[0] if goal_msg.yaw else 0.0,
        goal_msg.look_forward,
        goal_msg.goal_to_follower,
    )

    # Keep process alive briefly to reduce packet-loss risk when starting fresh.
    rospy.sleep(0.2)


if __name__ == "__main__":
    main()
