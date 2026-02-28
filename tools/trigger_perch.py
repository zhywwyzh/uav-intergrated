#!/usr/bin/env python3
"""
Trigger perch module directly:
publish /perch_trigger as geometry_msgs/PoseStamped.
"""

import argparse

import rospy
from geometry_msgs.msg import PoseStamped


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


def main():
    parser = argparse.ArgumentParser(description="Trigger perch by publishing /perch_trigger")
    parser.add_argument("--trigger-topic", default="/perch_trigger")
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    args = parser.parse_args()

    rospy.init_node("trigger_perch", anonymous=True)

    if not topic_type_compatible(args.trigger_topic, "geometry_msgs/PoseStamped"):
        rospy.logerr("Pre-check failed. Abort.")
        return

    pub = rospy.Publisher(args.trigger_topic, PoseStamped, queue_size=10)
    rospy.sleep(0.3)

    msg = PoseStamped()
    msg.pose.position.x = float(args.x)
    msg.pose.position.y = float(args.y)
    msg.pose.position.z = float(args.z)
    msg.pose.orientation.w = 1.0

    repeat = max(1, int(args.repeat))
    rate = rospy.Rate(max(1.0, float(args.rate)))
    for _ in range(repeat):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Published %s (%d times)", args.trigger_topic, repeat)


if __name__ == "__main__":
    main()
