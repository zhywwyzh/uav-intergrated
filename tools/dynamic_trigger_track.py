#!/usr/bin/env python3
"""
Trigger track and publish a dynamic target odom trajectory.

Default behavior:
1) publish /track_trigger once
2) publish /target/odom and /track_ekf/ekf_odom for 10s
3) target moves at 0.5 m/s along +45 deg in XY plane
"""

import argparse
import copy
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

DYN_TRACK_TAG = "\033[34m[DYN-TRACK]\033[0m"


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


def build_trigger_msg():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0
    return msg


def unit_velocity(direction, heading_deg):
    if direction == "x":
        return 1.0, 0.0
    if direction == "y":
        return 0.0, 1.0
    if direction == "diag45":
        s2 = math.sqrt(2.0)
        return 1.0 / s2, 1.0 / s2

    rad = math.radians(float(heading_deg))
    return math.cos(rad), math.sin(rad)


def build_odom(base, frame_id):
    msg = copy.deepcopy(base)
    msg.header.stamp = rospy.Time.now()
    if frame_id:
        msg.header.frame_id = frame_id
    return msg


def main():
    parser = argparse.ArgumentParser(description="Trigger track with dynamic target odom")
    parser.add_argument("--trigger-topic", default="/track_trigger")
    parser.add_argument("--trigger-rate", type=float, default=5.0)
    parser.add_argument("--trigger-repeat", type=int, default=1)

    parser.add_argument("--target-topic", default="/target/odom")
    parser.add_argument("--odom-topic", default="/track_ekf/ekf_odom")
    parser.add_argument("--source-odom-topic", default="/unity_odom")
    parser.add_argument("--source-odom-timeout", type=float, default=10.0)
    parser.add_argument("--fallback-static-odom", dest="fallback_static_odom", action="store_true")
    parser.add_argument("--no-fallback-static-odom", dest="fallback_static_odom", action="store_false")
    parser.set_defaults(fallback_static_odom=True)

    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--speed", type=float, default=0.5, help="target speed in m/s")
    parser.add_argument(
        "--direction",
        choices=["x", "y", "diag45", "heading"],
        default="diag45",
        help="target movement direction in world XY",
    )
    parser.add_argument("--heading-deg", type=float, default=45.0, help="used only when --direction heading")

    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--start-offset-x", type=float, default=5.0)
    parser.add_argument("--start-offset-y", type=float, default=0.0)
    parser.add_argument("--start-offset-z", type=float, default=0.0)
    args = parser.parse_args()

    rospy.init_node("dynamic_trigger_track", anonymous=True)
    rospy.loginfo("%s Trigger requested", DYN_TRACK_TAG)

    checks = [
        topic_type_compatible(args.trigger_topic, "geometry_msgs/PoseStamped"),
        topic_type_compatible(args.target_topic, "nav_msgs/Odometry"),
        topic_type_compatible(args.odom_topic, "nav_msgs/Odometry"),
    ]
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort.")
        return

    trigger_pub = rospy.Publisher(args.trigger_topic, PoseStamped, queue_size=10)
    target_pub = rospy.Publisher(args.target_topic, Odometry, queue_size=20)
    odom_pub = rospy.Publisher(args.odom_topic, Odometry, queue_size=20)

    odom_cache = OdomCache()
    rospy.Subscriber(args.source_odom_topic, Odometry, odom_cache.callback, queue_size=20)
    rospy.sleep(0.3)

    use_static_odom = False
    static_odom = Odometry()
    static_odom.header.frame_id = "world"
    static_odom.pose.pose.orientation.w = 1.0

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

    x0 = target_base.pose.pose.position.x + float(args.start_offset_x)
    y0 = target_base.pose.pose.position.y + float(args.start_offset_y)
    z0 = target_base.pose.pose.position.z + float(args.start_offset_z)

    ux, uy = unit_velocity(args.direction, args.heading_deg)
    vx = float(args.speed) * ux
    vy = float(args.speed) * uy

    rospy.loginfo(
        "%s Dynamic target start=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, 0.000), duration=%.2fs",
        DYN_TRACK_TAG,
        x0,
        y0,
        z0,
        vx,
        vy,
        float(args.duration),
    )

    trigger_period = 1.0 / max(1.0, float(args.trigger_rate))
    trigger_msg = build_trigger_msg()
    for _ in range(max(1, int(args.trigger_repeat))):
        if rospy.is_shutdown():
            return
        trigger_msg.header.stamp = rospy.Time.now()
        trigger_pub.publish(trigger_msg)
        rospy.sleep(trigger_period)

    rate = rospy.Rate(max(1.0, float(args.publish_rate)))
    t0 = rospy.Time.now().to_sec()
    duration = max(0.0, float(args.duration))
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        if t > duration:
            break

        if use_static_odom:
            odom_msg = copy.deepcopy(static_odom)
        else:
            odom_msg = copy.deepcopy(odom_cache.latest)
        odom_msg = build_odom(odom_msg, args.frame_id)

        target_msg = Odometry()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = args.frame_id
        target_msg.pose.pose.position.x = x0 + vx * t
        target_msg.pose.pose.position.y = y0 + vy * t
        target_msg.pose.pose.position.z = z0
        target_msg.pose.pose.orientation.w = 1.0
        target_msg.twist.twist.linear.x = vx
        target_msg.twist.twist.linear.y = vy

        odom_pub.publish(odom_msg)
        target_pub.publish(target_msg)
        rate.sleep()

    rospy.loginfo("%s Done.", DYN_TRACK_TAG)


if __name__ == "__main__":
    main()
