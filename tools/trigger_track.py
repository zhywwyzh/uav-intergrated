#!/usr/bin/env python3
"""
Trigger track module directly.

Default behavior:
1) publish /track_trigger continuously
2) publish fake YOLO + odom continuously (so track can run without external detector/odom)

Set --no-fake-inputs to only send trigger messages.
"""

import argparse
import copy
import random

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from object_detection_msgs.msg import BoundingBox, BoundingBoxes


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


def build_bbox(width, height, cls_name, prob):
    cx = width * 0.5 + random.uniform(-0.1, 0.1) * width
    cy = height * 0.5 + random.uniform(-0.1, 0.1) * height
    bw = width * 0.12
    bh = height * 0.18

    box = BoundingBox()
    box.probability = float(prob)
    box.xmin = int(max(0, cx - bw * 0.5))
    box.ymin = int(max(0, cy - bh * 0.5))
    box.xmax = int(min(width - 1, cx + bw * 0.5))
    box.ymax = int(min(height - 1, cy + bh * 0.5))
    box.id = 0
    box.Class = cls_name
    return box


def build_trigger_msg():
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.0
    msg.pose.orientation.w = 1.0
    return msg


def main():
    parser = argparse.ArgumentParser(description="Trigger track with optional fake inputs")
    parser.add_argument("--trigger-topic", default="/track_trigger")
    parser.add_argument("--trigger-rate", type=float, default=5.0)

    parser.add_argument("--fake-inputs", dest="fake_inputs", action="store_true")
    parser.add_argument("--no-fake-inputs", dest="fake_inputs", action="store_false")
    parser.set_defaults(fake_inputs=True)

    parser.add_argument("--yolo-topic", default="/yolov5trt/bboxes_pub")
    parser.add_argument("--odom-topic", default="/ekf/ekf_odom")
    parser.add_argument("--source-odom-topic", default="/unity_odom")
    parser.add_argument("--source-odom-timeout", type=float, default=10.0)
    parser.add_argument("--fallback-static-odom", dest="fallback_static_odom", action="store_true")
    parser.add_argument("--no-fallback-static-odom", dest="fallback_static_odom", action="store_false")
    parser.set_defaults(fallback_static_odom=True)

    parser.add_argument("--rate", type=float, default=5.0, help="fake input publish rate")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; <=0 means run until Ctrl+C")
    parser.add_argument("--frame-id", default="camera_link")
    parser.add_argument("--class-name", default="origin")
    parser.add_argument("--prob", type=float, default=1.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    rospy.init_node("trigger_track", anonymous=True)

    checks = [topic_type_compatible(args.trigger_topic, "geometry_msgs/PoseStamped")]
    if args.fake_inputs:
        checks.extend(
            [
                topic_type_compatible(args.yolo_topic, "object_detection_msgs/BoundingBoxes"),
                topic_type_compatible(args.odom_topic, "nav_msgs/Odometry"),
            ]
        )
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort.")
        return

    trigger_pub = rospy.Publisher(args.trigger_topic, PoseStamped, queue_size=10)
    yolo_pub = rospy.Publisher(args.yolo_topic, BoundingBoxes, queue_size=10)
    odom_pub = rospy.Publisher(args.odom_topic, Odometry, queue_size=20)

    odom_cache = OdomCache()
    rospy.Subscriber(args.source_odom_topic, Odometry, odom_cache.callback, queue_size=20)
    rospy.sleep(0.3)

    use_static_odom = False
    static_odom = Odometry()
    static_odom.header.frame_id = "world"
    static_odom.pose.pose.orientation.w = 1.0

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

    rospy.loginfo("Publishing track trigger on %s at %.2f Hz", args.trigger_topic, args.trigger_rate)
    if args.fake_inputs:
        rospy.loginfo(
            "Publishing fake track inputs to %s and %s at %.2f Hz (source odom: %s)",
            args.yolo_topic,
            args.odom_topic,
            args.rate,
            args.source_odom_topic,
        )

    end_time = rospy.Time.now().to_sec() + float(args.duration) if float(args.duration) > 0.0 else None
    loop_rate_hz = max(1.0, float(max(args.trigger_rate, args.rate if args.fake_inputs else 0.0)))
    loop_rate = rospy.Rate(loop_rate_hz)
    trigger_period = 1.0 / max(1.0, float(args.trigger_rate))
    fake_period = 1.0 / max(1.0, float(args.rate))
    last_trigger = 0.0
    last_fake = 0.0

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        if end_time is not None and now >= end_time:
            break

        if now - last_trigger >= trigger_period:
            msg = build_trigger_msg()
            msg.header.stamp = rospy.Time.now()
            trigger_pub.publish(msg)
            last_trigger = now

        if args.fake_inputs and now - last_fake >= fake_period:
            yolo_msg = BoundingBoxes()
            yolo_msg.header.stamp = rospy.Time.now()
            yolo_msg.header.frame_id = args.frame_id
            yolo_msg.image_header = yolo_msg.header
            yolo_msg.bounding_boxes = [build_bbox(args.width, args.height, args.class_name, args.prob)]

            if use_static_odom:
                odom_msg = copy.deepcopy(static_odom)
            else:
                odom_msg = copy.deepcopy(odom_cache.latest)
            odom_msg.header.stamp = rospy.Time.now()

            yolo_pub.publish(yolo_msg)
            odom_pub.publish(odom_msg)
            last_fake = now

        loop_rate.sleep()


if __name__ == "__main__":
    main()
