#! /usr/bin/env python
#coding: UTF-8

#subscirber_tutorial.pyとこれを使ってどのトピックがサブスクライブされているのか確認

import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher')
pub = rospy.Publisher('counter', Int32, queue_size = 10)

rate = rospy.Rate(1) #1Hz
count = 0
while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()