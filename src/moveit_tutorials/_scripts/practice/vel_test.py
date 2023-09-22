#!/usr/bin/env python
#coding: UTF-8

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
#just in case
import math

def main(flg):
    flag = flg.data
    # print(flag)
    vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
    #flagをsubscribeして、コールバックの中で速度をpublishしてるのでrateはpublisher側(vel_test_flag.py)で設定したレート通り
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.layout.data_offset = 1
    if flag == 1:
        print(flag)
        msg.data = [0.0, 0.0, 0.0, -0.08, 0.0, 0.0]
        vel_pub.publish(msg)
    if flag == 0:
        print(flag)
        msg.data = [0.0, 0.0, 0.0, 0.08, 0.0, 0.0]
        vel_pub.publish(msg)
    if flag == 2:
        print('stop')
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vel_pub.publish(msg)
        
if __name__ == '__main__':
    try:
        rospy.init_node('vel_flag_subscriber')
        rospy.loginfo('velocity control node started')
        sub = rospy.Subscriber('vel_flag', Int32, main)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


### publish するときの引数の書き方？？