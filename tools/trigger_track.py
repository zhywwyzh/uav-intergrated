#!/usr/bin/env python3
"""
Trigger track module directly.

Default behavior:
1) publish mode=2 to /uav_planner/trigger
2) publish /tracker_trigger once
3) publish fake target-odom + ego-odom continuously

Set --no-fake-inputs to only send trigger messages.
"""

import argparse
import copy

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

TRACK_TAG = "\033[34m[TRACK]\033[0m"


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


class OdomCache:
    def __init__(self):
        self.latest = None

    def callback(self, msg):
        self.latest = msg


def build_target_odom(base_odom, frame_id, dx, dy, dz):
    msg = copy.deepcopy(base_odom)
    msg.header.stamp = rospy.Time.now()
    if frame_id:
        msg.header.frame_id = frame_id
    msg.pose.pose.position.x += float(dx)
    msg.pose.pose.position.y += float(dy)
    msg.pose.pose.position.z += float(dz)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.linear.y = 0.0
    msg.twist.twist.linear.z = 0.0
    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = 0.0
    return msg


def build_trigger_msg():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0
    return msg


def main():
    parser = argparse.ArgumentParser(description="Trigger track with optional fake inputs")
    parser.add_argument("--mode-topic", default="/uav_planner/trigger")
    parser.add_argument("--mode-value", type=int, default=2)
    parser.add_argument("--trigger-topic", default="/tracker_trigger")
    parser.add_argument("--trigger-rate", type=float, default=5.0)
    parser.add_argument("--trigger-repeat", type=int, default=1)

    parser.add_argument("--fake-inputs", dest="fake_inputs", action="store_true")
    parser.add_argument("--no-fake-inputs", dest="fake_inputs", action="store_false")
    parser.set_defaults(fake_inputs=True)

    parser.add_argument("--yolo-topic", default="/target/odom")
    parser.add_argument("--odom-topic", default="/drone_0_visual_slam/odom")
    parser.add_argument("--source-odom-topic", default="/unity_odom")
    parser.add_argument("--source-odom-timeout", type=float, default=10.0)
    parser.add_argument("--fallback-static-odom", dest="fallback_static_odom", action="store_true")
    parser.add_argument("--no-fallback-static-odom", dest="fallback_static_odom", action="store_false")
    parser.set_defaults(fallback_static_odom=True)

    parser.add_argument("--rate", type=float, default=5.0, help="fake input publish rate")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; <=0 means run until Ctrl+C")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--target-offset-x", type=float, default=10.0)
    parser.add_argument("--target-offset-y", type=float, default=0.0)
    parser.add_argument("--target-offset-z", type=float, default=1.0)
    args = parser.parse_args()

    rospy.init_node("trigger_track", anonymous=True)
    rospy.loginfo("%s Trigger requested", TRACK_TAG)

    checks = [
        topic_type_compatible(args.mode_topic, "std_msgs/Int32"),
        topic_type_compatible(args.trigger_topic, "geometry_msgs/PoseStamped"),
    ]
    if args.fake_inputs:
        checks.extend(
            [
                topic_type_compatible(args.yolo_topic, "nav_msgs/Odometry"),
                topic_type_compatible(args.odom_topic, "nav_msgs/Odometry"),
            ]
        )
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort.")
        return

    mode_pub = rospy.Publisher(args.mode_topic, Int32, queue_size=10)
    trigger_pub = rospy.Publisher(args.trigger_topic, PoseStamped, queue_size=10)
    yolo_pub = rospy.Publisher(args.yolo_topic, Odometry, queue_size=20)
    odom_pub = rospy.Publisher(args.odom_topic, Odometry, queue_size=20)

    odom_cache = OdomCache()
    rospy.Subscriber(args.source_odom_topic, Odometry, odom_cache.callback, queue_size=20)
    rospy.sleep(0.3)

    use_static_odom = False
    static_odom = Odometry()
    static_odom.header.frame_id = "world"
    static_odom.pose.pose.orientation.w = 1.0
    fixed_target_odom = None

    if args.fake_inputs:
        start_wait = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and odom_cache.latest is None:
            if rospy.Time.now().to_sec() - start_wait > float(args.source_odom_timeout):
                if args.fallback_static_odom:
                    rospy.logwarn(
                        "Timeout waiting odom from %s, fallback to static odom.",
                        args.source_odom_topic,
                    )
                    use_static_odom = True
                    break
                rospy.logerr("Timeout waiting odom from %s", args.source_odom_topic)
                return
            rospy.sleep(0.1)
        if use_static_odom:
            target_base = copy.deepcopy(static_odom)
        else:
            target_base = copy.deepcopy(odom_cache.latest)
        if args.frame_id:
            target_base.header.frame_id = args.frame_id
        fixed_target_odom = build_target_odom(
            target_base,
            args.frame_id,
            args.target_offset_x,
            args.target_offset_y,
            args.target_offset_z,
        )
        rospy.loginfo(
            "%s Fixed absolute target set to (%.3f, %.3f, %.3f) in frame %s",
            TRACK_TAG,
            fixed_target_odom.pose.pose.position.x,
            fixed_target_odom.pose.pose.position.y,
            fixed_target_odom.pose.pose.position.z,
            fixed_target_odom.header.frame_id,
        )

    rospy.loginfo(
        "%s Publishing track trigger on %s (%d times at %.2f Hz)",
        TRACK_TAG,
        args.trigger_topic,
        max(1, int(args.trigger_repeat)),
        args.trigger_rate,
    )
    rospy.loginfo(
        "%s Publishing mode trigger %d on %s",
        TRACK_TAG,
        int(args.mode_value),
        args.mode_topic,
    )
    if args.fake_inputs:
        rospy.loginfo(
            "%s Publishing fake track inputs to %s and %s at %.2f Hz (source odom: %s)",
            TRACK_TAG,
            args.yolo_topic,
            args.odom_topic,
            args.rate,
            args.source_odom_topic,
        )

    end_time = rospy.Time.now().to_sec() + float(args.duration) if float(args.duration) > 0.0 else None
    loop_rate_hz = max(1.0, float(args.rate if args.fake_inputs else 10.0))
    loop_rate = rospy.Rate(loop_rate_hz)
    trigger_period = 1.0 / max(1.0, float(args.trigger_rate))
    fake_period = 1.0 / max(1.0, float(args.rate))
    last_fake = 0.0

    mode_msg = Int32(data=int(args.mode_value))
    for _ in range(3):
        if rospy.is_shutdown():
            return
        mode_pub.publish(mode_msg)
        rospy.sleep(0.03)

    trigger_msg = build_trigger_msg()
    for _ in range(max(1, int(args.trigger_repeat))):
        if rospy.is_shutdown():
            return
        trigger_msg.header.stamp = rospy.Time.now()
        trigger_pub.publish(trigger_msg)
        rospy.sleep(trigger_period)

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        if end_time is not None and now >= end_time:
            break

        if args.fake_inputs and now - last_fake >= fake_period:
            if use_static_odom:
                odom_msg = copy.deepcopy(static_odom)
            else:
                odom_msg = copy.deepcopy(odom_cache.latest)
            odom_msg.header.stamp = rospy.Time.now()
            if args.frame_id:
                odom_msg.header.frame_id = args.frame_id

            yolo_msg = copy.deepcopy(fixed_target_odom)
            yolo_msg.header.stamp = rospy.Time.now()

            yolo_pub.publish(yolo_msg)
            odom_pub.publish(odom_msg)
            last_fake = now

        loop_rate.sleep()


if __name__ == "__main__":
    main()
