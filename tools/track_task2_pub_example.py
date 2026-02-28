#!/usr/bin/env python3
"""
Atomic task+input publisher for task_id=2 testing.

Workflow:
1) Pre-check topic type compatibility.
2) Wait for /unity_odom (current pose source).
3) Publish /task_id once (default: 2).
4) Publish YOLO bbox + odom at 5 Hz continuously.
"""

import argparse
import copy

import rospy
from nav_msgs.msg import Odometry
from object_detection_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Int32


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
        "Topic %s type mismatch: current=%s expected=%s",
        topic_name,
        current_type,
        expected_type,
    )
    return False


def build_origin_bbox(frame_id, cls_name):
    """Build a simple origin bbox for quick testing."""
    msg = BoundingBoxes()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.image_header = msg.header

    box = BoundingBox()
    box.probability = 1.0
    box.xmin = 0
    box.ymin = 0
    box.xmax = 1
    box.ymax = 1
    box.id = 0
    box.Class = cls_name
    msg.bounding_boxes = [box]
    return msg


class UnityOdomCache:
    def __init__(self):
        self.latest = None

    def callback(self, msg):
        self.latest = msg


def main():
    parser = argparse.ArgumentParser(description="Publish task_id and 5Hz track test inputs")
    parser.add_argument("--task-id", type=int, default=2, help="task id to publish first")
    parser.add_argument("--task-topic", default="/task_id")
    parser.add_argument("--yolo-topic", default="/yolov5trt/bboxes_pub")
    parser.add_argument("--odom-topic", default="/ekf/ekf_odom")
    parser.add_argument("--unity-odom-topic", default="/unity_odom")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--frame-id", default="camera_link")
    parser.add_argument("--class-name", default="origin")
    parser.add_argument("--unity-timeout", type=float, default=10.0)
    args = parser.parse_args()

    rospy.init_node("track_task2_pub_example", anonymous=True)

    odom_cache = UnityOdomCache()
    rospy.Subscriber(args.unity_odom_topic, Odometry, odom_cache.callback, queue_size=20)

    task_pub = rospy.Publisher(args.task_topic, Int32, queue_size=10)
    yolo_pub = rospy.Publisher(args.yolo_topic, BoundingBoxes, queue_size=10)
    odom_pub = rospy.Publisher(args.odom_topic, Odometry, queue_size=20)

    rospy.sleep(0.5)

    checks = [
        check_topic_type_compatible(args.task_topic, "std_msgs/Int32"),
        check_topic_type_compatible(args.yolo_topic, "object_detection_msgs/BoundingBoxes"),
        check_topic_type_compatible(args.odom_topic, "nav_msgs/Odometry"),
    ]
    if not all(checks):
        rospy.logerr("Pre-check failed. Abort: publish neither task nor track inputs.")
        return

    start = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and odom_cache.latest is None:
        if rospy.Time.now().to_sec() - start > args.unity_timeout:
            rospy.logerr("Timeout waiting for %s. Abort publishing.", args.unity_odom_topic)
            return
        rospy.sleep(0.1)

    task_msg = Int32()
    task_msg.data = int(args.task_id)
    task_pub.publish(task_msg)
    rospy.loginfo("Published %s: %d", args.task_topic, task_msg.data)

    rate = rospy.Rate(args.rate)
    rospy.loginfo(
        "Publishing at %.2f Hz: %s (origin bbox), %s (from %s)",
        args.rate,
        args.yolo_topic,
        args.odom_topic,
        args.unity_odom_topic,
    )

    while not rospy.is_shutdown():
        yolo_msg = build_origin_bbox(args.frame_id, args.class_name)

        odom_msg = copy.deepcopy(odom_cache.latest)
        odom_msg.header.stamp = rospy.Time.now()

        yolo_pub.publish(yolo_msg)
        odom_pub.publish(odom_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
