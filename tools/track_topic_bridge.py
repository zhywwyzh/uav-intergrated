#!/usr/bin/env python3
"""
Bridge external detector/odom topics to tracker-specific topics.

Default mapping:
- /yolov5trt/bboxes_pub -> /track_object_topic
- /ekf/ekf_odom         -> /track_odom_topic
"""

import argparse

import rospy
from nav_msgs.msg import Odometry
from object_detection_msgs.msg import BoundingBoxes


def topic_type_compatible(topic_name, expected_type):
    """Return True when topic is unpublished or type matches expectation."""
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


class TrackTopicBridge:
    def __init__(self, source_yolo_topic, source_odom_topic, track_object_topic, track_odom_topic):
        self._object_pub = rospy.Publisher(track_object_topic, BoundingBoxes, queue_size=20)
        self._odom_pub = rospy.Publisher(track_odom_topic, Odometry, queue_size=50)

        self._got_yolo = False
        self._got_odom = False

        self._source_yolo_topic = source_yolo_topic
        self._source_odom_topic = source_odom_topic

        rospy.Subscriber(source_yolo_topic, BoundingBoxes, self._yolo_cb, queue_size=20)
        rospy.Subscriber(source_odom_topic, Odometry, self._odom_cb, queue_size=50)

    def _yolo_cb(self, msg):
        self._object_pub.publish(msg)
        if not self._got_yolo:
            self._got_yolo = True
            rospy.loginfo("Received first YOLO message from %s", self._source_yolo_topic)

    def _odom_cb(self, msg):
        self._odom_pub.publish(msg)
        if not self._got_odom:
            self._got_odom = True
            rospy.loginfo("Received first odom message from %s", self._source_odom_topic)


def main():
    parser = argparse.ArgumentParser(description="Bridge YOLO/Odom topics for Elastic-Tracker")
    parser.add_argument("--source-yolo-topic", default="/yolov5trt/bboxes_pub")
    parser.add_argument("--source-odom-topic", default="/ekf/ekf_odom")
    parser.add_argument("--track-object-topic", default="/track_object_topic")
    parser.add_argument("--track-odom-topic", default="/track_odom_topic")
    args = parser.parse_args()

    rospy.init_node("track_topic_bridge", anonymous=False)

    checks = [
        topic_type_compatible(args.source_yolo_topic, "object_detection_msgs/BoundingBoxes"),
        topic_type_compatible(args.source_odom_topic, "nav_msgs/Odometry"),
        topic_type_compatible(args.track_object_topic, "object_detection_msgs/BoundingBoxes"),
        topic_type_compatible(args.track_odom_topic, "nav_msgs/Odometry"),
    ]
    if not all(checks):
        rospy.logerr("Topic type pre-check failed. Bridge will not start.")
        return

    TrackTopicBridge(
        source_yolo_topic=args.source_yolo_topic,
        source_odom_topic=args.source_odom_topic,
        track_object_topic=args.track_object_topic,
        track_odom_topic=args.track_odom_topic,
    )

    rospy.loginfo(
        "Track bridge active: %s -> %s, %s -> %s",
        args.source_yolo_topic,
        args.track_object_topic,
        args.source_odom_topic,
        args.track_odom_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
