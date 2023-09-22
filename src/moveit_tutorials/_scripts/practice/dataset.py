#!/usr/bin/env python
#coding: UTF-8

###service action通信使わずにpython main関数だけでデータセット作成プログラムつくったやつ
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import datetime
import time

import random
import numpy as np
import sys
from math import pi
import moveit_commander
# import joint_planner2
from time import sleep

i = 0
count = 0

def MovetoGoal():
	theta = []
	now = datetime.datetime.now()
	filename1 = './result/' + now.strftime('%Y%m%d_%H%M%S') + '_joint_goal_result.txt'
	with open(filename1, "a") as f:
		####go to mokuhyo pose####
		# initialize MoveitCommander
		moveit_commander.roscpp_initialize(sys.argv)

		# prepare MoveGroupCommander
		move_group = moveit_commander.MoveGroupCommander("manipulator")

		# set max velocity
		move_group.set_max_velocity_scaling_factor(value=0.1)
		move_group.set_max_acceleration_scaling_factor(value=0.1)

		# set joint to define goal state
		joint_goal = [1.5708, -0.9225, 0.5934, -1.2217, -1.5708, 0.0]
		move_group.set_joint_value_target(joint_goal)

		# plan motion plan and execute
		move_group.go(wait=True)

		#stop
		move_group.stop()

		print('moved to mokuhyo pose')

		theta.append(joint_goal)

		print(theta)
		f.write(str(theta))
	f.close()

def MovetoRandomGoal():
	theta=[]
	moveit_commander.roscpp_initialize(sys.argv)
	move_group = moveit_commander.MoveGroupCommander("manipulator")
	move_group.set_max_velocity_scaling_factor(value=0.1)
	move_group.set_max_acceleration_scaling_factor(value=0.1)
	joint_goal = [1.5708, random.uniform(-0.95, -0.90), random.uniform(0.58, 0.62), random.uniform(-1.24, -1.20), -1.5708, 0.0]
	move_group.set_joint_value_target(joint_goal)
	move_group.go(wait=True)
	move_group.stop()
	print('moved to random pose')


def savecapturedImage(CI):
	now = datetime.datetime.now()
	ImageName = './data/' + now.strftime('%Y%m%d_%H%M%S') + '_gray_image.png'
	cv2.imwrite(ImageName, CI)
	print('ima totta')
	cv2.waitKey(1)

def MoveandCapture(data):
	global count
	bridge = CvBridge()
	orig = bridge.imgmsg_to_cv2(data, "bgr8")
	drawImg = orig
	resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
	drawImg = resized
	# joint_planner2.MovetoGoal()
	MovetoGoal()
	sleep(5)
	print('ugoita')
	count += 1
	print('count:%s' %(count))
	#if count is odd number
	if count % 2 != 0:
		savecapturedImage(drawImg)
		count += 1
		print('satsueishita')
		print('count:')
		print(count)
	else:
		print('count:')
		print(count)
		print('satsueishitenai')
 
	# for i in range(5):
	# 	print(i)
			# ##for loop haittekara NameError nanoka,hairumae nanoka print (i)dekakunin
			# joint_planner2.MovetoRandomGoal()
			# count += 1
			# print('random ni ugoita')
			# print('random %s' % (count))
			# if count % 2 != 0:
			# 	savecapturedImage(drawImg)
			# 	count += 1
			# 	print('random satsuei')
			# 	print(count)
		 
	# except Exception as err: 
	# 	print('err') 
	# 	#show error type
	# 	print(type(err)) 

def MoveRandomlyandCapture(data):
	global count
	bridge = CvBridge()
	orig = bridge.imgmsg_to_cv2(data, "bgr8")
	drawImg = orig
	resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
	drawImg = resized
	# print("kokomade")
	# joint_planner2.MovetoRandomGoal()
	MovetoRandomGoal()
	sleep(5)
	print('randomly ugoita')
	count += 1
	# print('random_count:%f', % (count))
	print('random count:')
	print('count')
	if count % 2 != 0:
		savecapturedImage(drawImg)
		count += 1
		print('satsueishita2')
		# print('count:%s', %(count))
		print("count:")
		print(count)
	else:
		# print('count:%s', %(count))
		print("count:")
		print(count)
		print('satsueishitenai')

def callback(msg):
	global i
	global count
	# if data == msg:
	# 	print('err')
	# 	break
	data = msg
	# print(i)
	try:	
		print(i)	
		if i == 0:
			print("haireta")
			MoveandCapture(data)
			i += 1
			print('i:')
			print(i)

		elif 1 <= i <= 6:
			print("randomhaireta")
			MoveRandomlyandCapture(data)
			i += 1
			print('i:')
			print(i)
		else:
			print('end')
			pass

	except Exception as err:
		print('err')
		print(type(err))

def start_node():
	global i
	global count
	rospy.loginfo('dataset node started')
	rospy.Subscriber("/camera/color/image_raw", Image, callback)
	print("Subscribernuketa")
	# rospy.Subscriber("/camera/color/image_raw", Image, MoveandCapture)
	# rospy.Subscriber("/camera/color/image_raw", Image, MoveRandomlyandCapture)
	rospy.spin()
	###subscribe shitekara for loop ha dekinaikamoshirenai
	###2kaime ikou 

if __name__ == '__main__':
	try:
		rospy.init_node('dataset')
		start_node()
	except rospy.ROSInterruptException:
		pass
