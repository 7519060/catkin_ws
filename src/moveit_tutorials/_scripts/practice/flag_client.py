#!/usr/bin/env python
# coding: UTF-8

import rospy
import actionlib
from std_msgs.msg import Int32
from moveit_tutorials.msg import FlagAction, FlagGoal
from time import sleep

##### フラグ変数を0と1を交互にflag_serverにgoalとして送信する。
##### serverからはフラグ変数を元にorientation +0.1と-0.1が交互に出力される

def client(flag):
    client = actionlib.SimpleActionClient('flag', FlagAction)
    client.wait_for_server()

    goal = FlagGoal()
    goal.flag = flag

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('flag_client')

    flag = 0
    while not rospy.is_shutdown():
        client(flag)
        sleep(1)
        flag = 1 - flag  # 0と1を交互に切り替える