#! /usr/bin/env python
#coding: UTF-8

import rospy
from std_msgs.msg import Int32
from time import sleep

#------------------------------------------
def callback(msg):
    # sleep(10)
    print(msg.data)
    
rospy.init_node('topic_subscriber')
print('onara')
sub = rospy.Subscriber('counter', Int32, callback)
print('unchi buri buri')
rospy.spin()
#普通に順番に購読される
#------------------------------------------

#------------------------------------------
# def cb(msg):
#     print('cb')
#     print(msg.data)
    
# while not rospy.is_shutdown():
#     rospy.init_node('topic_subscriber')
#     print('hi')
#     sleep(3)
#     print('hi 3 sec later')
#     sub = rospy.Subscriber('counter', Int32, cb)
#     sleep(1)
###rospy.Subscriberの行を実行後、print('hi')が実行されてる最中にもサブスクライブし続けていた
#------------------------------------------
    
#------------------------------------------
# while not rospy.is_shutdown():
#     # rospy.init_node('topic_subscriber')
#     print('hi')
#     sleep(3)
#     print('waited for 3 sec')
#     msg = rospy.wait_for_message('/counter', Int32, timeout=None)
#     rospy.loginfo(msg.data)
#     print(msg.data)
#     print('heard message')
###wait for message がなんんかうまく行かない　よくわからん

# #------------------------------------------
# def cb(msg):
#     print(msg.data)
    
# for i in range (10):
#     rospy.init_node('sub_for_10')
#     sub = rospy.Subscriber('counter', Int32, cb)
#     sleep(1)