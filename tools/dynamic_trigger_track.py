#!/usr/bin/env python3
"""
Trigger track and publish a dynamic target odom trajectory.

Default behavior:
1) publish mode=2 to /uav_planner/trigger
2) publish /tracker_trigger once
3) publish /target/odom and /track_ekf/ekf_odom for 10s
4) target moves at 0.5 m/s along +45 deg in XY plane
5) optional: refresh mode=2 during publish by --mode-refresh-rate
"""

import argparse
import copy
import math

import rospy
from rosgraph import Master
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

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


class Int32Cache:
    def __init__(self):
        self.latest = None

    def callback(self, msg):
        self.latest = int(msg.data)


def topic_subscriber_count(topic_name):
    try:
        master = Master(rospy.get_name())
        _, _, system_state = master.getSystemState()
        # system_state: [publishers, subscribers, services]
        subscribers = system_state[1] if len(system_state) > 1 else []
        for topic, nodes in subscribers:
            if topic == topic_name:
                return len(nodes)
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logwarn("Failed to query system state: %s", exc)
    return 0


def resolve_odom_topic(preferred_topic):
    candidates = [preferred_topic, "/ekf/ekf_odom", "/track_ekf/ekf_odom"]
    checked = []
    for topic in candidates:
        if topic in checked:
            continue
        checked.append(topic)
        if topic_subscriber_count(topic) > 0:
            return topic
    return preferred_topic


def log_odom_topic_state(topic_name, label):
    rospy.loginfo("%s %s: %s (subscribers=%d)", DYN_TRACK_TAG, label, topic_name, topic_subscriber_count(topic_name))


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
    parser.add_argument("--mode-topic", default="/uav_planner/trigger")
    parser.add_argument("--mode-value", type=int, default=2)
    parser.add_argument("--mode-repeat", type=int, default=1)
    parser.add_argument(
        "--mode-refresh-rate",
        type=float,
        default=0.0,
        help="republish mode trigger during dynamic publishing; <=0 disables refresh",
    )
    parser.add_argument("--mode-switch-topic", default="")
    parser.add_argument("--exit-on-mode-switch", dest="exit_on_mode_switch", action="store_true")
    parser.add_argument("--no-exit-on-mode-switch", dest="exit_on_mode_switch", action="store_false")
    parser.set_defaults(exit_on_mode_switch=True)
    parser.add_argument("--trigger-topic", default="/tracker_trigger")
    parser.add_argument("--trigger-rate", type=float, default=5.0)
    parser.add_argument("--trigger-repeat", type=int, default=1)

    parser.add_argument("--target-topic", default="/target/odom")
    parser.add_argument("--odom-topic", default="/drone_0_visual_slam/odom")
    parser.add_argument("--auto-resolve-odom-topic", dest="auto_resolve_odom_topic", action="store_true")
    parser.add_argument("--no-auto-resolve-odom-topic", dest="auto_resolve_odom_topic", action="store_false")
    parser.set_defaults(auto_resolve_odom_topic=False)
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
    parser.add_argument("--start-offset-x", type=float, default=6.0)
    parser.add_argument("--start-offset-y", type=float, default=0.0)
    parser.add_argument("--start-offset-z", type=float, default=0.0)
    args = parser.parse_args()

    rospy.init_node("dynamic_trigger_track", anonymous=True)
    rospy.loginfo("%s Trigger requested", DYN_TRACK_TAG)
    requested_odom_topic = args.odom_topic
    resolved_odom_topic = requested_odom_topic
    if args.auto_resolve_odom_topic:
        resolved_odom_topic = resolve_odom_topic(requested_odom_topic)
        if resolved_odom_topic != requested_odom_topic:
            rospy.logwarn(
                "Switch odom publish topic from %s to %s (has active subscribers).",
                requested_odom_topic,
                resolved_odom_topic,
            )
            rospy.logwarn(
                "Also mirroring odom to requested topic %s to avoid startup race.",
                requested_odom_topic,
            )
    args.odom_topic = resolved_odom_topic
    log_odom_topic_state(requested_odom_topic, "requested odom topic")
    if requested_odom_topic != args.odom_topic:
        log_odom_topic_state(args.odom_topic, "resolved odom topic")

    checks = [
        topic_type_compatible(args.mode_topic, "std_msgs/Int32"),
        topic_type_compatible(args.trigger_topic, "geometry_msgs/PoseStamped"),
        topic_type_compatible(args.target_topic, "nav_msgs/Odometry"),
        topic_type_compatible(args.odom_topic, "nav_msgs/Odometry"),
    ]
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort.")
        return

    mode_pub = rospy.Publisher(args.mode_topic, Int32, queue_size=10)
    trigger_pub = rospy.Publisher(args.trigger_topic, PoseStamped, queue_size=10)
    target_pub = rospy.Publisher(args.target_topic, Odometry, queue_size=20)
    odom_pub = rospy.Publisher(args.odom_topic, Odometry, queue_size=20)
    odom_pub_requested = None
    if requested_odom_topic != args.odom_topic:
        odom_pub_requested = rospy.Publisher(requested_odom_topic, Odometry, queue_size=20)

    odom_cache = OdomCache()
    rospy.Subscriber(args.source_odom_topic, Odometry, odom_cache.callback, queue_size=20)
    mode_cache = Int32Cache()
    mode_switch_topic = args.mode_switch_topic if args.mode_switch_topic else args.mode_topic
    if args.exit_on_mode_switch:
        rospy.Subscriber(mode_switch_topic, Int32, mode_cache.callback, queue_size=20)
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
    if float(args.mode_refresh_rate) > 0.0:
        rospy.loginfo(
            "%s Refresh mode=%d at %.2f Hz on %s",
            DYN_TRACK_TAG,
            int(args.mode_value),
            float(args.mode_refresh_rate),
            args.mode_topic,
        )

    trigger_period = 1.0 / max(1.0, float(args.trigger_rate))
    trigger_msg = build_trigger_msg()
    mode_msg = Int32(data=int(args.mode_value))
    for _ in range(max(1, int(args.mode_repeat))):
        if rospy.is_shutdown():
            return
        mode_pub.publish(mode_msg)
        rospy.sleep(0.03)
    for _ in range(max(1, int(args.trigger_repeat))):
        if rospy.is_shutdown():
            return
        trigger_msg.header.stamp = rospy.Time.now()
        trigger_pub.publish(trigger_msg)
        rospy.sleep(trigger_period)

    rate = rospy.Rate(max(1.0, float(args.publish_rate)))
    mode_refresh_period = None
    last_mode_refresh = 0.0
    if float(args.mode_refresh_rate) > 0.0:
        mode_refresh_period = 1.0 / float(args.mode_refresh_rate)

    t0 = rospy.Time.now().to_sec()
    duration = max(0.0, float(args.duration))
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        if t > duration:
            break

        if (
            args.exit_on_mode_switch
            and mode_cache.latest is not None
            and int(mode_cache.latest) != int(args.mode_value)
        ):
            rospy.logwarn(
                "%s Detected mode switch on %s: current=%d expected=%d, stop dynamic publishing.",
                DYN_TRACK_TAG,
                mode_switch_topic,
                int(mode_cache.latest),
                int(args.mode_value),
            )
            break

        if mode_refresh_period is not None and (t - last_mode_refresh >= mode_refresh_period):
            mode_msg.data = int(args.mode_value)
            mode_pub.publish(mode_msg)
            last_mode_refresh = t

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
        if odom_pub_requested is not None:
            odom_pub_requested.publish(odom_msg)
        target_pub.publish(target_msg)
        rate.sleep()

    rospy.loginfo("%s Done.", DYN_TRACK_TAG)


if __name__ == "__main__":
    main()
