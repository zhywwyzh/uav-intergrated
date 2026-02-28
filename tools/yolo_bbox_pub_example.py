#!/usr/bin/env python3
import argparse
import random

import rospy
from object_detection_msgs.msg import BoundingBox, BoundingBoxes


def build_bbox(width, height, cls, prob):
    cx = width * 0.5 + random.uniform(-0.15, 0.15) * width
    cy = height * 0.5 + random.uniform(-0.15, 0.15) * height
    bw = width * 0.15
    bh = height * 0.20

    xmin = int(max(0, cx - bw * 0.5))
    xmax = int(min(width - 1, cx + bw * 0.5))
    ymin = int(max(0, cy - bh * 0.5))
    ymax = int(min(height - 1, cy + bh * 0.5))

    box = BoundingBox()
    box.probability = prob
    box.xmin = xmin
    box.ymin = ymin
    box.xmax = xmax
    box.ymax = ymax
    box.id = 0
    box.Class = cls
    return box


def main():
    parser = argparse.ArgumentParser(description="Publish YOLO BoundingBoxes for target_ekf testing")
    parser.add_argument("--topic", default="/yolov5trt/bboxes_pub")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--class-name", default="target")
    parser.add_argument("--prob", type=float, default=0.9)
    parser.add_argument("--frame-id", default="camera_link")
    args, _ = parser.parse_known_args()

    rospy.init_node("yolo_bbox_pub_example", anonymous=True)
    pub = rospy.Publisher(args.topic, BoundingBoxes, queue_size=10)
    rate = rospy.Rate(args.rate)

    rospy.loginfo("publishing %s at %.2f Hz", args.topic, args.rate)

    while not rospy.is_shutdown():
        msg = BoundingBoxes()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = args.frame_id
        msg.image_header = msg.header
        msg.bounding_boxes = [build_bbox(args.width, args.height, args.class_name, args.prob)]
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
