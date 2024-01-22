#!/usr/bin/env python
#coding: UTF-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# import time
from time import sleep
import  numpy as np
import math
import statistics
# I_dsr_vec = np.empty((1105920, 1)) ###for UI
I_dsr_vec = np.empty((2073600, 1))
rsme_data = []
# nop = 1105920 ###for UI
nop = 2073600 ###realsense
def gen_I_dsr_vec():
    global I_dsr_vec
    I_dsr_orig = cv2.imread('./data/desired_im.png')
    # I_dsr_orig = cv2.imread('./input_dsrim/kensyo_desired_image.png')
    I_dsr_gry = cv2.cvtColor(I_dsr_orig, cv2.COLOR_BGR2GRAY)
    I_dsr_arr = np.array(I_dsr_gry, dtype='float64')
    I_dsr_vec = I_dsr_arr.reshape(-1,1)
    return I_dsr_vec 

def process_image(msg):
    global I_dsr_vec
    global rsme_data
    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
    arr = np.array(gray, dtype = 'float64')
    vec = arr.reshape(-1,1)
    #画像偏差
    dI = I_dsr_vec - vec ###目標-現在
    dI2 = dI**2
    Isum = np.sum(dI2)
    rsme = math.sqrt(Isum / nop)
    rsme_data.append(rsme)
    return rsme_data
            
def start_node():
    rospy.loginfo('rsme average node started')
    for i in range(10):
        rospy.Subscriber('/camera/color/image_raw', Image, process_image)
        # rospy.Subscriber('/camera/image_raw', Image, process_image) #for UI camera
        sleep(1)
    rsme_ave = statistics.mean(rsme_data)
    print(rsme_ave)
    rsme_var = statistics.variance(rsme_data)
    print(rsme_var)


if __name__ == '__main__':
    try:
        rospy.init_node('kensyo_desired_image')
        gen_I_dsr_vec()
        start_node()
    except rospy.ROSInterruptException:
        pass


# rospy.init_node('sub_one')
# image_raw = rospy.wait_for_message('/camera/color/image', Image)
# bridge = CvBridge()
# bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8')
# cv2.imwrite('./input_dsrim/kensyo_desired_image.png', bgr) ###なんかこれできない
# print('saved')


