#!/usr/bin/env python3
"""
Trigger ego module directly:
1) publish mode=1 to /uav_planner/trigger
2) call planning service (/drone_0_ego_planner_node/planning/enable)
3) publish one GoalSet trigger to /goal_with_id_from_station
"""

import argparse

import rospy
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import GoalSet
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

EGO_TAG = "\033[32m[EGO]\033[0m"


def topic_type_compatible(topic_name, expected_type):
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
        "Topic %s type mismatch: current=%s expected=%s",
        topic_name,
        current_type,
        expected_type,
    )
    return False


def build_goalset(x, y, z, yaw, look_forward):
    msg = GoalSet()
    msg.to_drone_ids = [0]

    pt = Point()
    pt.x = float(x)
    pt.y = float(y)
    pt.z = float(z)

    msg.goal = [pt]
    msg.yaw = [float(yaw)]
    msg.look_forward = bool(look_forward)
    msg.goal_to_follower = False
    return msg


def main():
    parser = argparse.ArgumentParser(description="Enable/disable ego planning and publish GoalSet")
    parser.add_argument("--mode-topic", default="/uav_planner/trigger")
    parser.add_argument("--mode-value", type=int, default=1)
    parser.add_argument("--planning-service", default="/drone_0_ego_planner_node/planning/enable")
    parser.add_argument("--goal-topic", default="/goal_with_id_from_station")
    parser.add_argument("--goal-delay", type=float, default=0.3)
    parser.add_argument("--x", type=float, default=3.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=1.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--look-forward", action="store_false")
    parser.add_argument("--disable-planning", action="store_true")
    args = parser.parse_args()

    rospy.init_node("trigger_ego", anonymous=True)

    checks = [
        topic_type_compatible(args.mode_topic, "std_msgs/Int32"),
        topic_type_compatible(args.goal_topic, "quadrotor_msgs/GoalSet"),
    ]
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort.")
        return

    mode_pub = rospy.Publisher(args.mode_topic, Int32, queue_size=10)
    goal_pub = rospy.Publisher(args.goal_topic, GoalSet, queue_size=10)
    rospy.sleep(0.3)

    planning_target = not args.disable_planning
    rospy.loginfo("%s Trigger requested, planning_target=%s", EGO_TAG, planning_target)
    mode_msg = Int32(data=int(args.mode_value))
    for _ in range(3):
        if rospy.is_shutdown():
            return
        mode_pub.publish(mode_msg)
        rospy.sleep(0.03)
    try:
        rospy.wait_for_service(args.planning_service, timeout=2.0)
        set_planning = rospy.ServiceProxy(args.planning_service, SetBool)
        resp = set_planning(planning_target)
        rospy.loginfo(
            "Service %s called with data=%s, success=%s, message=%s",
            args.planning_service,
            planning_target,
            resp.success,
            resp.message,
        )
    except (rospy.ServiceException, rospy.ROSException) as exc:
        rospy.logwarn("Failed to call planning service %s: %s", args.planning_service, exc)

    rospy.sleep(max(0.0, float(args.goal_delay)))

    goal_msg = build_goalset(args.x, args.y, args.z, args.yaw, args.look_forward)
    goal_pub.publish(goal_msg)
    rospy.loginfo(
        "%s Published GoalSet to %s: (%.3f, %.3f, %.3f), yaw=%.3f, look_forward=%s",
        EGO_TAG,
        args.goal_topic,
        goal_msg.goal[0].x,
        goal_msg.goal[0].y,
        goal_msg.goal[0].z,
        goal_msg.yaw[0],
        goal_msg.look_forward,
    )

    rospy.sleep(0.2)


if __name__ == "__main__":
    main()
