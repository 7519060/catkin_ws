#! /usr/bin/env python
#coding: UTF-8

import rospy
import actionlib

from std_srvs.srv import Empty

from time import sleep

from moveit_tutorials.msg import EmptyAction, EmptyGoal


def MotionCommand():
	motion_goal = EmptyGoal()
	print('motion goal generated')
	motion_client.wait_for_server()
	print('waiting for motion server')
	motion_client.send_goal(motion_goal)
	print('motion goal message sent')
	motion_client.wait_for_result()
	print('got result from motion node')
	motion_result = motion_client.get_result()

def RandomMotionCommand():
	motion_goal2 = EmptyGoal()
	print('random motion goal generated')
	motion_client2.wait_for_server()
	print('waiting for random motion server')
	motion_client2.send_goal(motion_goal2)
	print('random motion goal message sent')
	motion_client2.wait_for_result()
	print('got result from random motion node')
	motion_result2 = motion_client2.get_result()

# def capture_command():
#     rospy.wait_for_service('tore')
#     print('caoture request sent')
#     responce = capture_client()
#     print('got responce from capture node')

if __name__ == '__main__':
	rospy.init_node('moiton_and_capture_manager')
	motion_client = actionlib.SimpleActionClient('ugoke', EmptyAction)
	motion_client2 = actionlib.SimpleActionClient('ugoke_random', EmptyAction)
	capture_client = rospy.ServiceProxy('tore', Empty)
	MotionCommand()
	# sleep(0.5)
	# capture_command()
	for i in range(560):
		print('i= ')
		print(i)
		RandomMotionCommand()
		# sleep(0.5)
		# capture_command()