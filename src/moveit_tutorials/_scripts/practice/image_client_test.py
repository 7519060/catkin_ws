#! /usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import cv2
import time
# from cv_bridge import CvBridge
from math import pi
from time import sleep
import math
from moveit_tutorials.srv import GetCurrentImageData

I_vec = np.empty((2073600,1), dtype = 'float64')
def receive_image():
    global I_vec
    rospy.wait_for_service('kure')
    image_client = rospy.ServiceProxy('kure', GetCurrentImageData)
    response = image_client()
    # print(type(response.current_image)) ###type tuple
    I_vec_arr = np.array(response.current_image, dtype='float64')
    print(I_vec_arr.dtype)
    I_vec = I_vec_arr.reshape(-1,1)
    print(I_vec.dtype)
    return response.current_image, I_vec

def calRSME():
    global I_vec
    I_dsr_orig = cv2.imread('./orig.png')
    I_dsr = cv2.cvtColor(I_dsr_orig, cv2.COLOR_BGR2GRAY)
    I_dsr_arr = np.array(I_dsr, dtype='float64')
    I_dsr_vec = I_dsr_arr.reshape(-1,1) 
    dI = I_vec - I_dsr_vec
    dI2 = dI**2
    Isum = np.sum(dI2)
    rsme = math.sqrt(Isum / 2073600)
    print('RSME = %f' % rsme)

    
if __name__ == '__main__':
    rospy.init_node('test')
    start_time = time.time()
    receive_image()
    calRSME()
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(elapsed_time)
