#!/usr/bin/env python
#coding: UTF-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import datetime
import time
from std_srvs.srv import Empty, EmptyResponse

drawImg = None
# def showliveImage(liveimg):
# 	cv2.imshow('live image',liveimg)
# 	cv2.waitKey(1)

def savecapturedImage(CI):
	now = datetime.datetime.now()
	ImageName = './data/' + now.strftime('%Y%m%d_%H%M%S') + '_image.png'
	cv2.imwrite(ImageName, CI)
	print('saved image')
	cv2.waitKey(1)

#topicとして流れているimage_rawをcv2の形式に変換
#リサイズやグレー化はここでは行わない、あくまで生データのみを出力
def henkan(msg):
	global drawImg
	try: 
		bridge = CvBridge()
		drawImg = bridge.imgmsg_to_cv2(msg, "bgr8") ###for realsense
		# drawImg = bridge.imgmsg_to_cv2(msg, "mono8") ###for ui camera
	except Exception as err: 
		print ('err') 
	# showliveImage(drawImg)
	return drawImg

def CaptureCommand(res):
	global drawImg
	rospy.loginfo('command node started')
	savecapturedImage(drawImg)
	return EmptyResponse()

def start_node():
	rospy.loginfo('ready to serve')
	rospy.Subscriber("/camera/color/image_raw", Image, henkan)
	# rospy.Subscriber("/camera/image_raw", Image, henkan) #for UI camera
	rospy.Service('tore', Empty, CaptureCommand)
	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('capture_server')
		start_node()
	except rospy.ROSInterruptException:
		pass
