#!/usr/bin/env python
#coding: UTF-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def process_image(msg):
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    # orig = bridge.imgmsg_to_cv2(msg, "mono8") ###for UI camera
    cv2.imshow('last image', orig)
    # cv2.waitKey(0)
    if cv2.waitKey(1) & 0xff == 27:
        cv2.imwrite('./servo_data/kensyo_last_image.png', orig)
        print ('ok')
    
def start_node():
	rospy.loginfo('kensyo last image node started')
	rospy.Subscriber("/camera/color/image_raw", Image, process_image)
	# rospy.Subscriber("/camera/image_raw", Image, process_image) #for UI camera
	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('kensyo_last_image')
		start_node()
	except rospy.ROSInterruptException:
		pass
