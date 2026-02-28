#!/usr/bin/env python3
"""
Unified trigger arbiter with global preemption.

Behavior:
1) Receive external trigger (ego/track/perch)
2) Immediately publish global preempt and emergency-stop
3) Disable ego planning
4) Forward only the latest trigger to the target module internal topic
"""

import argparse
import copy
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import GoalSet
from std_msgs.msg import Empty
from std_srvs.srv import SetBool


class TriggerArbiter:
    def __init__(self, args):
        self.args = args
        self._token_lock = threading.Lock()
        self._service_lock = threading.Lock()
        self._latest_token = 0
        self._set_planning = None

        self.preempt_pub = rospy.Publisher(args.preempt_topic, Empty, queue_size=10)
        self.emergency_pub = rospy.Publisher(args.emergency_stop_topic, Empty, queue_size=10)

        self.ego_out_pub = rospy.Publisher(args.ego_out_topic, GoalSet, queue_size=10)
        self.ego_out_legacy_pub = None
        if args.ego_out_legacy_topic:
            self.ego_out_legacy_pub = rospy.Publisher(args.ego_out_legacy_topic, GoalSet, queue_size=10)
        self.track_out_pub = rospy.Publisher(args.track_out_topic, PoseStamped, queue_size=10)
        self.perch_out_pub = rospy.Publisher(args.perch_out_topic, PoseStamped, queue_size=10)

        for topic in self._split_topics(args.ego_in_topics):
            rospy.Subscriber(topic, GoalSet, self._ego_cb, queue_size=10)
            rospy.loginfo("Arbiter subscribed ego trigger topic: %s", topic)
        rospy.Subscriber(args.track_in_topic, PoseStamped, self._track_cb, queue_size=20)
        rospy.Subscriber(args.perch_in_topic, PoseStamped, self._perch_cb, queue_size=20)

    @staticmethod
    def _split_topics(csv_text):
        return [s.strip() for s in csv_text.split(",") if s.strip()]

    def _next_token(self):
        with self._token_lock:
            self._latest_token += 1
            return self._latest_token

    def _is_latest(self, token):
        with self._token_lock:
            return token == self._latest_token

    def _ego_cb(self, msg):
        self._schedule("ego", msg)

    def _track_cb(self, msg):
        self._schedule("track", msg)

    def _perch_cb(self, msg):
        self._schedule("perch", msg)

    def _schedule(self, kind, msg):
        token = self._next_token()
        rospy.loginfo("[ARBITER] New trigger: %s (token=%d)", kind, token)
        worker = threading.Thread(
            target=self._process_trigger,
            args=(token, kind, copy.deepcopy(msg)),
            daemon=True,
        )
        worker.start()

    def _process_trigger(self, token, kind, msg):
        self._publish_preempt()
        self._set_ego_planning(False)
        if not self._is_latest(token):
            return

        rospy.sleep(max(0.0, float(self.args.preempt_settle_sec)))
        if not self._is_latest(token):
            return

        if kind == "ego":
            self._set_ego_planning(True)
            if not self._is_latest(token):
                return
            self._forward_ego(token, msg)
            return

        if kind == "track":
            self._forward_pose(token, msg, self.track_out_pub, self.args.track_out_topic)
            return

        self._forward_pose(token, msg, self.perch_out_pub, self.args.perch_out_topic)

    def _publish_preempt(self):
        preempt_msg = Empty()
        for _ in range(max(1, int(self.args.preempt_burst_count))):
            self.preempt_pub.publish(preempt_msg)
            self.emergency_pub.publish(preempt_msg)
            rospy.sleep(0.01)
        rospy.loginfo("[ARBITER] Published preempt + emergency stop.")

    def _set_ego_planning(self, enable):
        with self._service_lock:
            try:
                if self._set_planning is None:
                    rospy.wait_for_service(self.args.ego_planning_service, timeout=0.5)
                    self._set_planning = rospy.ServiceProxy(self.args.ego_planning_service, SetBool)
                self._set_planning(bool(enable))
            except (rospy.ROSException, rospy.ServiceException):
                pass

    def _forward_ego(self, token, msg):
        repeat = max(1, int(self.args.forward_repeat))
        interval = max(0.0, float(self.args.forward_interval_sec))
        for _ in range(repeat):
            if not self._is_latest(token):
                return
            self.ego_out_pub.publish(msg)
            if self.ego_out_legacy_pub is not None and self.args.ego_out_legacy_topic != self.args.ego_out_topic:
                self.ego_out_legacy_pub.publish(msg)
            if interval > 0.0:
                rospy.sleep(interval)
        if self.ego_out_legacy_pub is not None and self.args.ego_out_legacy_topic != self.args.ego_out_topic:
            rospy.loginfo(
                "[ARBITER] Forwarded ego trigger -> %s and %s (x%d)",
                self.args.ego_out_topic,
                self.args.ego_out_legacy_topic,
                repeat,
            )
        else:
            rospy.loginfo("[ARBITER] Forwarded ego trigger -> %s (x%d)", self.args.ego_out_topic, repeat)

    def _forward_pose(self, token, msg, pub, topic_name):
        repeat = max(1, int(self.args.forward_repeat))
        interval = max(0.0, float(self.args.forward_interval_sec))
        for _ in range(repeat):
            if not self._is_latest(token):
                return
            if hasattr(msg, "header"):
                msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            if interval > 0.0:
                rospy.sleep(interval)
        rospy.loginfo("[ARBITER] Forwarded trigger -> %s (x%d)", topic_name, repeat)


def parse_args():
    parser = argparse.ArgumentParser(description="Global trigger preempt arbiter")
    parser.add_argument("--ego-in-topics", default="/ego_trigger")
    parser.add_argument("--ego-out-topic", default="/goal_with_id_from_station")
    parser.add_argument("--ego-out-legacy-topic", default="")
    parser.add_argument("--track-in-topic", default="/track_trigger")
    parser.add_argument("--track-out-topic", default="/track_trigger_internal")
    parser.add_argument("--perch-in-topic", default="/perch_trigger")
    parser.add_argument("--perch-out-topic", default="/perch_trigger_internal")
    parser.add_argument("--preempt-topic", default="/mix/preempt")
    parser.add_argument("--emergency-stop-topic", default="/command/emergency_stop")
    parser.add_argument("--ego-planning-service", default="/drone_0_ego_planner_node/planning/enable")
    parser.add_argument("--preempt-settle-sec", type=float, default=0.08)
    parser.add_argument("--preempt-burst-count", type=int, default=1)
    parser.add_argument("--forward-repeat", type=int, default=5)
    parser.add_argument("--forward-interval-sec", type=float, default=0.03)
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("task_trigger_arbiter", anonymous=False)
    TriggerArbiter(args)
    rospy.loginfo("Trigger arbiter started.")
    rospy.spin()


if __name__ == "__main__":
    main()
