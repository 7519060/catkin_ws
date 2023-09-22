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
    cv2.imshow('initial image', orig)
    if cv2.waitKey(1) & 0xff == 27:
        cv2.imwrite('./servo_data/kensyo_initial_image.png', orig)
        print ('ok')
    
def start_node():
	rospy.loginfo('kensyo intial image node started')
	rospy.Subscriber("/camera/color/image_raw", Image, process_image)
	# rospy.Subscriber("/camera/image_raw", Image, process_image) #for UI camerea
	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('kensyo_init_image')
		start_node()
	except rospy.ROSInterruptException:
		pass


# rospy.init_node('sub_one')
# image_raw = rospy.wait_for_message('/camera/color/image', Image)
# bridge = CvBridge()
# bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8')
# cv2.imwrite('./input_dsrim/kensyo_desired_image.png', bgr) ###なんかこれできない
# print('saved')
