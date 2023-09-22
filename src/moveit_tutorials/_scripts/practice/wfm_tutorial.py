#!/usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# rospy.init_node('sub_one')
# image_raw = rospy.wait_for_message('/camera/color/image_raw', Image)
# bridge = CvBridge()

# bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8')
# print(type(bgr))
# print(bgr.dtype)
# print(bgr.shape)
# gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
# print(type(gry))
# print(gry.dtype)
# print(gry.shape)
# gry_cvec = gry.reshape(-1,1)
# print(type(gry_cvec))
# print(gry_cvec.dtype)
# print(gry_cvec.shape)
# I_vec_arr = np.array(gry_cvec, dtype = 'float64')
# print(type(I_vec_arr))
# print(I_vec_arr.dtype)
# print(I_vec_arr.shape)
# I_vec = I_vec_arr.reshape(-1,1)
# print(type(I_vec))
# print(I_vec.dtype)
# print(I_vec.shape)

# bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8')
# print(type(bgr))
# print(bgr.dtype)
# print(bgr.shape)
# gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) ###これより先にfloat型への変換したらエラー
# print(type(gry))
# print(gry.dtype)
# print(gry.shape)
# gry2 = np.array(gry, dtype = 'float64')
# print(type(gry2))
# print(gry2.dtype)
# print(gry2.shape)
# gry_cvec = gry.reshape(-1,1)
# print(type(gry_cvec))
# print(gry_cvec.dtype)
# print(gry_cvec.shape)

# I_dsr = cv2.imread('./data/20230117_024517_image.png')
# I_dsr_gry = cv2.cvtColor(I_dsr, cv2.COLOR_BGR2GRAY)
# print(I_dsr_gry)

a = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12]])
a_vec = a.reshape(-1,1)
print(a)
print(a_vec)
