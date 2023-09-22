#! /usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import pandas as pd
import csv
import glob
import cv2
import time
from cv_bridge import CvBridge
import moveit_commander
import sys
from math import pi
from time import sleep
import math

#read csv data tutorial
# dir_path = './pinv_int_mat/*csv'
# csv_files = glob.glob(dir_path)
# print(csv_files)
# csvdata = pd.read_csv(str(csv_files[0]), header=None)
# # csvdata = pd.read_csv('./pinv_int_mat/pinv_int_mat.csv', header=None)
# pinv_int_mat = np.array
# pinv_int_mat = csvdata.values
# print(pinv_int_mat.shape)
# # pinv_int_mat_cvec = pinv_int_mat.reshape(-1,1)
# # print(pinv_int_mat_cvec.shape)

#各成分の和tutorial
# I = [[5,4,3,2,1], [1,2,3,4,5]]
# I2 = np.array([[5,4,3,2,1], [1,2,3,4,5]])
# print(I,I2)
# sum1 = np.sum(I)
# print(sum1)
# sum2 = np.sum(I2)
# print(sum2)

#wild card * no test
# while True:
#     im_path = './input_dsrim/*_image.png'
#     im = glob.glob(im_path)
#     I_dsr = cv2.imread(str(im[0]), cv2.IMREAD_GRAYSCALE)
#     # I_dsr = cv2.imread('./input_dsrim/20221208_204236_image.png', cv2.IMREAD_GRAYSCALE)
#     # resize = cv2.resize(I_dsr, None, fx=0.5, fy=0.5)
#     cv2.imshow('imshow test', I_dsr)
#     key = cv2.waitKey(10)
#     if key==27:
#         break
#wild card test kokomade

# ############read csv tutorial
# # mat_path = './pinv_int_mat/*csv'
# # csv_files = glob.glob(mat_path)
# # csvdata = pd.read_csv(str(csv_files[0]), header=None)
# readcsvtime = time.time() ###6 x 2073600の行列では216秒くらい
# # csvdata = pd.read_csv('./pinv_int_mat/pinv_int_mat.csv', header=None)
# # print('csvyometa')
# # ########pickle ni henkan
# # csvdata.to_pickle('./pinv_int_mat_pickle/pinv_int_mat.pickle')
# # # print('pickle ni henkan dekita')
# pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat.pickle')
# # # print('read pickle dekita')
# pinv_int_mat = np.array
# pinv_int_mat = pickledata.values
# # # print('read csv2pickle file')
# # #########csv no mama 
# # pinv_int_mat = np.array
# # pinv_int_mat = csvdata.values
# endreadcsvtime = time.time()
# elpsdt = endreadcsvtime - readcsvtime
# print(elpsdt)
# print('read csv file values')
# #########read csv kokomade

########RMSEをデータセットで計算した
# def calIth(): #calculate image threshold
#     dsrim_path = './input_dsrim/*_image.png'
#     dsrim_name = glob.glob(dsrim_path)
#     rndmim_path = './input_rndmim/*_image.png'
#     rndmim_name = glob.glob(rndmim_path)
#     filename = './output/threshold.txt'
#     RMSE = []
#     with open (filename, 'a') as f:
#         for i,img in enumerate(dsrim_name):
#             I_dsr = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
#             I_dsr_vec = I_dsr.reshape(-1,1)
#             # cv2.imwrite('./output/tutorial_' + str(i) + '.png', I_dsr)
#             for j,rimg in enumerate(rndmim_name):
#                 I_rndm = cv2.imread(rimg, cv2.IMREAD_GRAYSCALE)
#                 I_rndm_vec = I_rndm.reshape(-1,1)
#                 dI = I_dsr_vec - I_rndm_vec
#                 Ith = np.sum(dI)
#                 RMSE = math.sqrt(Ith / 2073600)
#                 RMSE.append(RMSE)
#                 # RMSE.extend(RMSE) ##float object is not iterableってエラーメッセージ
#             f.write(str(RMSE)) 
#     print('done')
# #     ################
def calRMSE():
    # I_dsr = cv2.imread('./input_dsrim/20230105_125655_image.png', cv2.IMREAD_GRAYSCALE)
    ###!!!cv2.IMREAD_GRAYSCALEとcv2.COLOR_BGR2GRAY 厳密には違うらしい
    ###!!!厳密な画素値使いたいならcv2.COLOR_BGR2GRAY 使ったほうがいいらしい
    # I_dsr = cv2.imread('./orig.png', cv2.IMREAD_GRAYSCALE)
    I_dsr_orig = cv2.imread('./data/desired_image.png')
    I_dsr = cv2.cvtColor(I_dsr_orig, cv2.COLOR_BGR2GRAY)
    I_dsr_cvec = I_dsr.reshape(-1,1)
    I_dsr_vec = np.array(I_dsr_cvec, dtype='float64')
    # I_rndm_jpg = cv2.imread('./frame0000.jpg')
    # cv2.imwrite('./test.png', I_rndm_jpg)
    # I_rndm = cv2.imread('./test.png', cv2.IMREAD_GRAYSCALE)
    I_rndm_orig = cv2.imread('./input_dsrim/kensyo_desired_image.png')
    I_rndm = cv2.cvtColor(I_rndm_orig, cv2.COLOR_BGR2GRAY)
    I_rndm_cvec = I_rndm.reshape(-1,1)
    I_rndm_vec = np.array(I_rndm_cvec, dtype='float64')
    dI = I_dsr_vec - I_rndm_vec
    dI2 = dI**2
    Ith = np.sum(dI2)
    RMSE = math.sqrt(Ith / 2073600)
    print('RMSE = %f' % RMSE)
    
# from sklearn import preprocessing
# def calRMSEnorm():
#     I_dsr = cv2.imread('./orig2.png')
#     I_dsr_gry = cv2.cvtColor(I_dsr, cv2.COLOR_BGR2GRAY)
#     I_dsr_arr = np.array(I_dsr_gry, dtype = 'float64')
#     I_dsr_cvec = I_dsr_arr.reshape(-1,1)
#     I_dsr_vec = preprocessing.minmax_scale(I_dsr_cvec)
    
#     I_rndm = cv2.imread('./orig.png')
#     I_rndm_gry = cv2.cvtColor(I_rndm, cv2.COLOR_BGR2GRAY)
#     I_rndm_arr = np.array(I_rndm_gry, dtype = 'float64')
#     I_rndm_cvec = I_rndm_arr.reshape(-1,1)
#     I_rndm_vec = preprocessing.minmax_scale(I_rndm_cvec)
    
#     dI = I_dsr_vec - I_rndm_vec
#     print(dI.shape)
#     dI2 = dI**2
#     Ith = np.sum(dI2)
#     RMSE = math.sqrt(Ith / 2073600)
#     print('RMSE = %f' % RMSE)

# def servo(data):
#     global pinv_int_mat
#     print('subscribed')
#     Ith = 500000000
#     print(Ith)
#     while not Ith <= 1000:
#         lmbd = 0.5
#         print(lmbd)
#         t = 0.03 #(sec) same as image_raw publish rate
#         print(t)
#         #generating desired image column vector
#         dsrim_path = './input_dsrim/*_image.png'
#         print('dsrim path')
#         dsrim_name = glob.glob(dsrim_path)
#         I_dsr = cv2.imread(str(dsrim_name[0]), cv2.IMREAD_GRAYSCALE)
#         print('dssr im yometa')
#         I_dsr_vec = I_dsr.reshape(-1,1)
#         print('dsrimvec gen')
        
#         #for moveit setups
#         print('moveit settei')
#         moveit_commander.roscpp_initialize(sys.argv)
#         move_group = moveit_commander.MoveGroupCommander('manipulator')
#         move_group.set_max_velocity_scaling_factor(value = 0.1)
#         move_group.set_max_acceleration_scaling_factor(value = 0.1)
#         print('moveit settei owari')

#         #generating live image column vector
#         print('imageraw henkan') 
#         bridge = CvBridge()
#         bgr = bridge.imgmsg_to_cv2(data, 'bgr8')
#         gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
#         I_vec = gry.reshape(-1,1)
#         print('keisanmae')

#         #calculating
#         # servo_goal=[] #計算結果のジョイント角列で出てきそうだけど大丈夫なのでしょうか
#         # send_servo_goal=[]
#         dI = I_dsr_vec - I_vec
#         print('keisan')
#         Ith = np.sum(dI)
#         print(pinv_int_mat.shape)
#         servo_vel = -lmbd*(np.dot(pinv_int_mat, dI))
#         print(servo_vel.shape)
#         servo_goal = t*servo_vel
#         print(servo_goal.shape)
#         print(servo_goal)
#         test_servo_goal = np.empty((6))##koreto
#         test_servo_goal = servo_goal[:,0]##korede shita no error kaiketsu
#         print(type(test_servo_goal))
#         print(test_servo_goal)
#         # send_servo_goal = servo_goal.reshape(-1, 1)
#         # print(send_servo_goal.shape)
#         # print(type(send_servo_goal))
#         # send_list_servo_goal = send_servo_goal.tolist()
#         # print(type(send_list_servo_goal))
#         # send_double_servo_goal = map(float, send_list_servo_goal)
#         # print(type(send_double_servo_goal))
#         # move_group.set_joint_value_target(send_double_servo_goal)############
#         move_group.set_joint_value_target(test_servo_goal)##kokoni c++ no kata janaito hairanaimitaina error
#         move_group.go(wait=True)
#         move_group.stop()
#         print('moved')

# # ------------------------
# ###moveitの設定を別へ
# ###Move()で初期位置へ、Move2()でジョイント角全0へ
# ###get current joint values をつかってみた
# move_group = None

# def MoveitSettings():
#     global move_group
#     moveit_commander.roscpp_initialize(sys.argv)
#     move_group = moveit_commander.MoveGroupCommander('manipulator')
#     move_group.set_max_velocity_scaling_factor(value=0.1)
#     move_group.set_max_acceleration_scaling_factor(value=0.1)
#     return move_group
    
# def Move():
#     global move_group
#     theta = []
#     # moveit_commander.roscpp_initialize(sys.argv) ###別の関数として上で定義
#     # move_group = moveit_commander.MoveGroupCommander("manipulator")
#     # move_group.set_max_velocity_scaling_factor(value=0.1)
#     # move_group.set_max_acceleration_scaling_factor(value=0.1)
#     # joint_goal = [random.uniform(1.55, 1.59), random.uniform(-0.94, -0.90), random.uniform(0.57, 0.61), random.uniform(-1.24, -1.20), random.uniform(-1.59, -1.55), 0.0]
#     joint_goal = np.array([1.57, -0.92, 0.59, -1.22, -1.57, 0.0]) ###listのままでは足し算できない(後ろに要素がたされるだけ)
#     move_group.set_joint_value_target(joint_goal)
#     print(type(joint_goal))
#     print(joint_goal.shape)
#     move_group.go(wait=True)
#     print('went')
#     move_group.stop()
#     print('moved to initial pose')
#     # theta.append(joint_goal)
#     # print(theta)
#     print(joint_goal)

# def Move2():
#     global move_group
#     d_joint_goal = np.array([-1.55, 0.90, -0.57, 1.20, 1.55, 0.0])
#     print('d_joint_goal')
#     print(d_joint_goal)
#     print(type(d_joint_goal))
#     print(d_joint_goal.shape)
#     current_joint_values = np.array(move_group.get_current_joint_values())###np.arrayにしておく
#     print('current_joint_values')
#     print(current_joint_values)
#     print(type(current_joint_values))
#     new_joint_goal = current_joint_values + d_joint_goal
#     print('new_joint_values')
#     print(new_joint_goal)
#     print(type(new_joint_goal))
#     move_group.set_joint_value_target(new_joint_goal)
#     print('set')
#     move_group.go(wait=True)
#     print('went2')
#     move_group.stop()
#     print('moved to new pose')     

# ######append tutorial
# x = np.array([1,2,3,4])
# y = np.array([5,6,7,8])
# z = np.append(x,y) 
# print(z)
# x = [1,2,3,4]
# y = [5,6,7,8]
# # x.append(y)
# x.extend(y)
# print(x)
# ###ここまでの結果は[1 2 3 4 5 6 7 8] [1, 2, 3, 4, 5, 6, 7, 8]になった
# ######append tutorial end

###np.dot掛け算確認
# a = np.array([[1,2],[3,4],[5,6]])
# print(a.shape)
# print(a)
# b = np.array([[1,2,3,4],[4,3,2,1]])
# print(b.shape)
# print(b)
# c = np.dot(a,b)
# print(c.shape)
# print(c)
# a2 = a**2 ###これで各要素2乗
# print(a2)
# b2 = b**2
# print(b2)
# d = np.array([[1],[2]])
# print(d.shape)
# print(d)
# e = np.dot(a,d)
# print(e.shape)
# print(e)
# f = np.array([1,2])
# print(f.shape)
# print(f)
# g = np.dot(a,f)
# print(g.shape)
# print(g)
# t = 0.01
# at = np.dot(t,a)
# at2 = t*a
# dt = np.dot(t,d)
# dt2 = t*d
# print(type(at)) ###typeは全部numpy.ndarray
# print(at.shape)
# print(at)
# print(type(at2))
# print(at2.shape)
# print(at2.shape)
# print(at2)
# print(type(dt))
# print(dt.shape)
# print(dt)
# print(type(dt2))
# print(dt2.shape)
# print(dt2)
###kekka
# (3, 2)
# [[1 2]
#  [3 4]
#  [5 6]]
# (2, 4)
# [[1 2 3 4]
#  [4 3 2 1]]
# (3, 4)
# [[ 9  8  7  6]
#  [19 18 17 16]
#  [29 28 27 26]]
# (2, 1)
# [[1]
#  [2]]
# (3, 1)
# [[ 5]
#  [11]
#  [17]]
# (2,)
# [1 2]
# (3,)
# [ 5 11 17]
###よって画像ヤコビアンと画像偏差の行列積は問題なさそう、、

###service 通信tutorial image_server.pyのclientとして
# from moveit_tutorials.srv import GetCurrentImageData

# def get_image():
#     print('waiting for send_image service server')
#     rospy.wait_for_service('kure')
#     print('send image server found')
#     image_client = rospy.ServiceProxy('kure', GetCurrentImageData)
#     print('send image request sent')
#     response = image_client()
#     print(type(response.current_image)) ###<type 'tuple'>
#     # print(response.current_image)
#     I_vec_arr = np.array(response.current_image)
#     print(type(I_vec_arr)) ###<type 'numpy.ndarray'>
#     print(I_vec_arr.shape) ###(2073600,)
#     I_vec_rshp = I_vec_arr.reshape(-1,1)
#     print(type(I_vec_rshp)) ###<type 'numpy.ndarray'>
#     print(I_vec_rshp.shape) ###(2073600, 1)
#     print(I_vec_rshp)
#     return response.current_image

# from moveit_tutorials.srv import GetCurrentImageData
# I_vec = np.empty((6,1))
# def image_client():
#     global I_vec
#     rospy.wait_for_service('kure')
#     image_client = rospy.ServiceProxy('kure', GetCurrentImageData)
#     response = image_client()
#     I_vec_arr = np.array(response.current_image)
#     I_vec = I_vec_arr.reshape(-1,1)
#     return response.current_image, I_vec

# def calRMSE():
#     global I_vec
#     I_dsr_orig = cv2.imread('./orig.png')
#     I_dsr = cv2.cvtColor(I_dsr_orig, cv2.COLOR_BGR2GRAY)
#     I_dsr_vec = I_dsr.reshape(-1,1)
#     dI = I_vec - I_dsr_vec
#     dI2 = dI**2
#     Isum = np.sum(dI2)
#     RMSE = math.sqrt(Isum / 2073600)
#     print('RMSE = %f' % RMSE)

    
if __name__ == '__main__':
    rospy.init_node('test')
    # start_time = time.time()
    # image_client()
    # calRMSE()
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # print(elapsed_time)
    # get_image()
    # calIth()
    calRMSE()
    # calRMSEnorm()
# #     # MoveitSettings()
# #     # print('kokomade')
# #     # Move()
# #     # print('kokomade2')
# #     # Move2()
# #     # print('kokomade3')
