#!/usr/bin/env python
#coding: UTF-8

import rospy
from moveit_tutorials.srv import GetCurrentCount, GetCurrentCountResponse
from std_msgs.msg import Int32
# from time import sleep

###publisher_tutotial.pyのトピックをsubscribeしてcount_client.pyにサービス通信によって値を渡す
num = Int32
def show_count(msg):
    global num
    print(msg.data)
    num = msg.data
    return num

def send_count(req): ###Empty requestに対してはreq不使用のまま(?)
    global num
    print('sent number')
    print('-----')
    print(num)
    print('-----')
    return GetCurrentCountResponse(count = num)

def sub_count():
    rospy.loginfo('ready to send num to client')
    rospy.Subscriber('counter', Int32, show_count)
    rospy.Service('kure', GetCurrentCount, send_count)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        rospy.init_node('count_server')
        sub_count()
    except rospy.ROSInterruptException:
        pass