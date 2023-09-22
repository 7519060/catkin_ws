#!/usr/bin/env python
#coding: UTF-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#np.array
# from PIL import Image
# import numpy as np

import datetime
import time

def showliveImage(liveimg):
    cv2.imshow('window no namae',liveimg)
    cv2.waitKey(1)

# def showImage(img):
# 	cv2.imshow('seishiga', img)
# 	cv2.waitKey(1)

def savecapturedImage(CI):
	now = datetime.datetime.now()
	ImageName = './data/' + now.strftime('%Y%m%d_%H%M%S') + '_gray_image.png'
	cv2.imwrite(ImageName, CI)
	print('k')
	cv2.waitKey(1)

def process_image(msg):
	try: 
		# convert sensor_msgs/Image to opencv image
		bridge = CvBridge()
		orig = bridge.imgmsg_to_cv2(msg, "bgr8")
		gry = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
		drawImg = gry
		
		# #resize image (half-size) for easier processing syori kousokuka
		# # resized = cv2.resize(orig, None, fx=0.125, fy=0.125)
		# resized = cv2.resize(orig, None, fx=0.5, fy=0.5)
		# drawImg = resized

		# #convert to single-channel image
		# gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
		# drawImg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

		if cv2.waitKey(1) & 0xff == 27:
			# cv2.imwrite('./data/rs__resized_image.png', resized)
			cv2.imwrite('./orig.png', orig)
			print ('ok')
			# print(orig.shape)
			# print(resized.shape)
		 
	except Exception as err: 
		print ('err') 
	
	# showliveImage(drawImg)
	showliveImage(drawImg)

def start_node():
	rospy.loginfo('opencv_tutorial2 node started')
	rospy.Subscriber("/camera/color/image_raw", Image, process_image)
	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('opencv_tutorial2')
		start_node()
	except rospy.ROSInterruptException:
		pass
