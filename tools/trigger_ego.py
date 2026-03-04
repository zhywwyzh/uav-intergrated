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
from std_msgs.msg import Empty, Int32
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


class Int32Cache:
    def __init__(self):
        self.latest = None

    def callback(self, msg):
        self.latest = int(msg.data)


def publish_mode_burst(pub, msg, repeat, interval):
    for _ in range(max(1, int(repeat))):
        if rospy.is_shutdown():
            return
        pub.publish(msg)
        rospy.sleep(max(0.0, float(interval)))


def wait_for_ego_handover(args, mode_pub, mode_msg, track_mode_cache, owner_cache):
    timeout = max(0.0, float(args.wait_switch_timeout))
    if timeout <= 0.0:
        return True

    refresh_period = None
    if float(args.mode_refresh_rate) > 0.0:
        refresh_period = 1.0 / float(args.mode_refresh_rate)
    last_refresh = -1e9
    start = rospy.Time.now().to_sec()
    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - start
        if elapsed > timeout:
            return False

        if refresh_period is not None and (elapsed - last_refresh >= refresh_period):
            mode_pub.publish(mode_msg)
            last_refresh = elapsed

        track_ready = track_mode_cache.latest is None or track_mode_cache.latest == 0
        if args.require_owner_ego:
            owner_ready = owner_cache.latest is None or owner_cache.latest == int(args.mode_value)
        else:
            owner_ready = True

        if track_ready and owner_ready:
            return True
        rate.sleep()

    return False


def main():
    parser = argparse.ArgumentParser(description="Enable/disable ego planning and publish GoalSet")
    parser.add_argument("--mode-topic", default="/uav_planner/trigger")
    parser.add_argument("--mode-value", type=int, default=1)
    parser.add_argument("--mode-repeat", type=int, default=3)
    parser.add_argument("--mode-interval", type=float, default=0.03)
    parser.add_argument(
        "--mode-refresh-rate",
        type=float,
        default=10.0,
        help="republish mode trigger while waiting handover; <=0 disables refresh",
    )
    parser.add_argument("--wait-switch-timeout", type=float, default=3.0)
    parser.add_argument("--track-mode-state-topic", default="/track/mode_state")
    parser.add_argument("--cmd-owner-topic", default="/uav_planner/cmd_owner")
    parser.add_argument("--require-owner-ego", dest="require_owner_ego", action="store_true")
    parser.add_argument("--no-require-owner-ego", dest="require_owner_ego", action="store_false")
    parser.set_defaults(require_owner_ego=True)
    parser.add_argument("--preempt-topic", default="/tracker_preempt")
    parser.add_argument("--preempt-repeat", type=int, default=2)
    parser.add_argument("--preempt-before-switch", dest="preempt_before_switch", action="store_true")
    parser.add_argument("--no-preempt-before-switch", dest="preempt_before_switch", action="store_false")
    parser.set_defaults(preempt_before_switch=True)
    parser.add_argument("--planning-service", default="/drone_0_ego_planner_node/planning/enable")
    parser.add_argument("--planning-service-timeout", type=float, default=0.8)
    parser.add_argument("--planning-service-retry", type=int, default=2)
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
    if args.preempt_before_switch:
        checks.append(topic_type_compatible(args.preempt_topic, "std_msgs/Empty"))
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort.")
        return

    mode_pub = rospy.Publisher(args.mode_topic, Int32, queue_size=10)
    goal_pub = rospy.Publisher(args.goal_topic, GoalSet, queue_size=10)
    preempt_pub = None
    if args.preempt_before_switch:
        preempt_pub = rospy.Publisher(args.preempt_topic, Empty, queue_size=10)

    track_mode_cache = Int32Cache()
    owner_cache = Int32Cache()
    rospy.Subscriber(args.track_mode_state_topic, Int32, track_mode_cache.callback, queue_size=20)
    rospy.Subscriber(args.cmd_owner_topic, Int32, owner_cache.callback, queue_size=20)
    rospy.sleep(0.3)

    planning_target = not args.disable_planning
    rospy.loginfo("%s Trigger requested, planning_target=%s", EGO_TAG, planning_target)
    if preempt_pub is not None:
        preempt_msg = Empty()
        for _ in range(max(1, int(args.preempt_repeat))):
            if rospy.is_shutdown():
                return
            preempt_pub.publish(preempt_msg)
            rospy.sleep(0.02)

    mode_msg = Int32(data=int(args.mode_value))
    publish_mode_burst(mode_pub, mode_msg, args.mode_repeat, args.mode_interval)

    switched = wait_for_ego_handover(args, mode_pub, mode_msg, track_mode_cache, owner_cache)
    if not switched:
        rospy.logwarn(
            "%s Handover wait timeout(%.2fs): track_mode=%s owner=%s. Continue to publish goal.",
            EGO_TAG,
            float(args.wait_switch_timeout),
            str(track_mode_cache.latest),
            str(owner_cache.latest),
        )
    else:
        rospy.loginfo(
            "%s Handover ready: track_mode=%s owner=%s",
            EGO_TAG,
            str(track_mode_cache.latest),
            str(owner_cache.latest),
        )

    for _ in range(max(1, int(args.planning_service_retry))):
        try:
            rospy.wait_for_service(args.planning_service, timeout=float(args.planning_service_timeout))
            set_planning = rospy.ServiceProxy(args.planning_service, SetBool)
            resp = set_planning(planning_target)
            rospy.loginfo(
                "Service %s called with data=%s, success=%s, message=%s",
                args.planning_service,
                planning_target,
                resp.success,
                resp.message,
            )
            break
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn("Failed to call planning service %s: %s", args.planning_service, exc)

    rospy.sleep(max(0.0, float(args.goal_delay)))

    goal_msg = build_goalset(args.x, args.y, args.z, args.yaw, args.look_forward)
    goal_pub.publish(goal_msg)
    mode_pub.publish(mode_msg)
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
