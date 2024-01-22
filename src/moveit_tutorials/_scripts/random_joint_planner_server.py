#!/usr/bin/env python
# coding: UTF-8

import rospy
import actionlib
from moveit_tutorials.msg import ValidJointsAction, ValidJointsResult

import sys 
import moveit_commander
# import csv
# import pandas as pd
        
def random_pose(goal):
    # theta = []
    ### ランダムな角度を固定したcsvファイル作成しているのでappendしてfileに書き込む必要ない
    ### pose_data[]取りたかったらjoint_planner_server2.py参照
    # move_group.set_max_velocity_scaling_factor(value=0.08)
    # move_group.set_max_acceleration_scaling_factor(value=0.08)
    joint_goal = list(goal.valid_joint_values)
    print(joint_goal)
    print(type(joint_goal))
    # joint_goal_list = joint_goal.tolist()
    # move_group.set_joint_value_target(joint_goal_list)
    move_group.set_joint_value_target(joint_goal)
    print('sent random joint goal')
    move_group.go(wait=True)
    print('moved to random pose')
    # theta.append(joint_goal)
    # filename = './rndmth_result/random_theta_result.csv'
    # with open (filename, 'a') as f:
    #     writer = csv.writer(f)
    #     writer.writerows(theta)
    random_result = ValidJointsResult()
    random_motion_server.set_succeeded(random_result)
    # moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.init_node('random_motion_server')
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    rospy.loginfo('random motion server started')
    random_motion_server = actionlib.SimpleActionServer('ugoke_random', ValidJointsAction, random_pose, False)
    random_motion_server.start()
    rospy.spin()
    

# class ValidJointsActionServer:
#     def __init__(self):
#         self.server = actionlib.SimpleActionServer('ugoke_random', ValidJointsAction, self.execute, False)
#         self.server.start()

#     def execute(self, goal):
#         # Print the received goal
#         print(goal.goal)

#         # Mark the action as completed
#         self.server.set_succeeded()

# if __name__ == '__main__':
#     rospy.init_node('random_motion_server')
#     server = ValidJointsActionServer()
#     rospy.spin()

