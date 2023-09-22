#!/usr/bin/env python
#coding: UTF-8

import rospy
from moveit_tutorials.srv import GetCurrentCount

def get_count():
    rospy.wait_for_service('kure')
    count_client = rospy.ServiceProxy('kure', GetCurrentCount)
    response = count_client()
    num = response.count
    print(num)
    return response.count

if __name__ == '__main__':
    rospy.init_node('count_client')
    get_count()
    