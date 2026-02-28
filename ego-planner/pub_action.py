#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import GoalSet

def main():
    rospy.init_node('goalset_sender')
    pub = rospy.Publisher('/goal_with_id_from_station', GoalSet, queue_size=10)
    rospy.sleep(1) 

    msg = GoalSet()
    msg.to_drone_ids = [0]

    pt = Point()
    pt.x = 1.22
    pt.y = 7.9
    pt.z = 1.5
    msg.goal = [pt]

    # 只给xyz，yaw用默认0
    msg.yaw = [1.57]
    msg.look_forward = False
    msg.goal_to_follower = False

    pub.publish(msg)
    rospy.loginfo("已发送GoalSet消息")

if __name__ == '__main__':
    main()