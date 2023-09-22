#! /usr/bin/env python
#coding: UTF-8

import rospy
import actionlib

from std_srvs.srv import Empty

from time import sleep

from moveit_tutorials.msg import EmptyAction, EmptyGoal

def MotionCommand():
	motion_client = actionlib.SimpleActionClient('ugoke', EmptyAction)
	motion_client.wait_for_server()
	print('waiting for motion server')
	motion_goal = EmptyGoal()
	print('motion goal generated')
	motion_client.send_goal(motion_goal)
	print('motion goal message sent')
	motion_client.wait_for_result()
	print('got result from motion node')
	motion_result = motion_client.get_result()

def RandomMotionCommand():
	motion_client2 = actionlib.SimpleActionClient('ugoke_random', EmptyAction)
	motion_client2.wait_for_server()
	print('waiting for random motion server')
	motion_goal2 = EmptyGoal()
	print('random motion goal generated')
	motion_client2.send_goal(motion_goal2)
	print('random motion goal message sent')
	motion_client2.wait_for_result()
	print('got result from random motion node')
	motion_result2 = motion_client2.get_result()

def CaptureCommand():
	print('waiting for capture service')
	rospy.wait_for_service('tore')
	print('capture service found')
	capture_client = rospy.ServiceProxy('tore', Empty)
	print('cap goal sent')
	responce = capture_client()
	print('got responce from cap node')
	print('end capture command client')	

def main():
	rospy.init_node('moiton_and_capture_manager')
	MotionCommand()
	# sleep(1)
	CaptureCommand()
	# sleep(1)
	for j in range(400):
		print('j =')
		print(j)
		RandomMotionCommand()
		# sleep(1)
		CaptureCommand()
		# sleep(1)


if __name__ == '__main__':
	main()