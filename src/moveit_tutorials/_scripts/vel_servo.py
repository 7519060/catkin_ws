#!/usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import cv2
import pandas as pd
import math
import time
import matplotlib.pyplot as plt
import csv
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge

#for velocity control
from std_msgs.msg import Float64MultiArray

#/joint_group_vel_controller のためには scaled_pos_joint_traj_controller を切らないといけない
#move_groupによるジョイント角取得もできないので /joint_states topicから角度取得する必要ある

#if分岐の位置、joint_statesの取得方法をトピック通信からサービス通信に変えたバージョンはvel_servo_v2.py

nop = 2073600 #number of pixels
pinv_int_mat_double = np.empty((6,nop))
I_dsr_vec = np.empty((nop, 1))
rmse = 100
lmbd = 0.2
rmse_data = [] 
time_series = []

base_joint_data = []
shoulder_joint_data = []
elbow_joint_data = []
wrist1_joint_data = []
wrist2_joint_data = []
wrist3_joint_data = []

joint_vel_values_data = []

# # desired_pose = [-0.10968, 0.372502, 0.236982]
# desired_pose = [-0.1558, 0.3379, 0.3698] #screwdriver
# pose_dist_data = []

def readpickle():
    global pinv_int_mat_double
    pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat_double.pickle')
    pinv_int_mat_double = pickledata.values
    print('read pseudo inverse interaction matrix in pickle')
    return pinv_int_mat_double

def gen_dsrimvec(): 
    global I_dsr_vec
    I_dsr = cv2.imread('./input_dsrim/kensyo_desired_image.png')
    I_dsr_gry = cv2.cvtColor(I_dsr, cv2.COLOR_BGR2GRAY)
    I_dsr_arr = np.array(I_dsr_gry, dtype = 'float64')
    I_dsr_vec = I_dsr_arr.reshape(-1,1)
    print('got column vector of grayscaled desired image')
    return I_dsr_vec

def get_joint_state():
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    global joint_vel_values_data
    #速度取得、各ジョイント_data = []配列に追加していく
    base_joint_data.append(JointState.velocity[0])
    shoulder_joint_data.append(JointState.velocity[1])
    elbow_joint_data.append(JointState.velocity[2])
    wrist1_joint_data.append(JointState.velocity[3])
    wrist2_joint_data.append(JointState.velocity[4])
    wrist3_joint_data.append(JointState.velocity[5])
    print('appended joint velocity values')
    #csvファイルに速度変化を記録
    #joint_vel_valuesに追加、あとでwrite csvする
    joint_vel_values_list = JointState.velocity.tolist()
    joint_vel_values_data.append(joint_vel_values_list)
    ###ここから右のやつ参考に配列にそのときのjointstatesを入れたものを生成、あとでコールバック関数抜けたところでグラフ作成
    
    
    ###callback関数内で数値取得したときreturn必要？
    return base_joint_data, shoulder_joint_data, elbow_joint_data, wrist1_joint_data, wrist2_joint_data, wrist3_joint_data, joint_vel_values_data

def servo(msg):
    global pinv_int_mat_double
    global lmbd
    global I_dsr_vec
    global nop
    global rmse
    global rmse_data
    global time_series
    
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    
    # vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
    
    image_raw = msg
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8') 
    gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gry2 = np.array(gry, dtype = 'float64')
    I_vec = gry2.reshape(-1,1)
    
    #画像偏差計算
    dI = I_dsr_vec - I_vec ###目標-現在
    dI2 = dI**2
    Isum = np.sum(dI2)
    rmse = math.sqrt(Isum / nop)
    print('rmse = %f' % rmse)

    #角速度計算
    # vel_input = Float64MultiArray()
    # # vel_input.data = lmbd*(np.dot(pinv_int_mat_double, dI)) 
    # # print(vel_input)
    # # # print(type(vel_input.data)) ###numpy.ndarrayでした
    # # print(vel_input.data.shape)
    # vel_input.layout.data_offset = 1
    
    vel_input.data = lmbd*(np.dot(pinv_int_mat_double, dI)) 
    # print(vel_input) ###layoutとdataからなる形でした
    # print(type(vel_input.data)) ###numpy.ndarrayでした
    # print(vel_input.data.shape) ###(6,1)でした
        
    vel_pub.publish(vel_input)
        
    #get_joint_state()　内で各ジョイントの速度取得+速度データを配列に追加
    rospy.Subscriber('/joint_states', JointState, get_joint_state)
    
    current_time = time.time() - start_time
    time_series.append(current_time)
    rmse_data.append(rmse)
    
    ####append忘れてるよ
    
    return rmse, time_series, rmse_data, base_joint_data, shoulder_joint_data, elbow_joint_data, wrist1_joint_data, wrist2_joint_data, wrist3_joint_data

def stop_servo():
    #各ジョイント速度0をpublishして停止させる
    # vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
    # vel_input = Float64MultiArray()
    # vel_input.layout.data_offset = 1
    vel_input.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    vel_pub.publish(vel_input)
    print('stop command sent')

def main():
    global rmse
    global rmse_data
    global time_series
    
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    
    global joint_vel_values_data
    
    ###下のdetaがprintされていないので初期値のrmseの値でずっと上のif分岐入ってる
    if rmse > 85:
        print('haitta')
        rospy.Subscriber('/camera/color/image_raw', Image, servo)
        print('deta')
    if rmse < 85:
        print('stop')
        stop_servo()
        filename = './servo_data/loop_rmse_data.csv'
        filename2 = './servo_data/joint_vel_data.csv'
        with open (filename, 'w') as f, open(filename2, 'w')as f2:
            writer = csv.writer(f)
            writer.writerow(rmse_data)
            writer2 = csv.writer(f2)
            writer2.writerow(joint_vel_values_data)

        #generate graph
        fig_rmse = plt.figure()
        plt.xlabel('Time')
        plt.ylabel('RMSE')
        plt.plot(time_series, rmse_data, 'b-')
        plt.grid()
        fig_rmse.savefig('./servo_data/rmse_double.png')
        
        #6つのグラフの配置
        fig = plt.figure(figsize = (10,10))
        ax1 = fig.add_subplot(3,2,1)
        ax2 = fig.add_subplot(3,2,2)
        ax3 = fig.add_subplot(3,2,3)
        ax4 = fig.add_subplot(3,2,4)
        ax5 = fig.add_subplot(3,2,5)
        ax6 = fig.add_subplot(3,2,6)
        
        #各ループごとの各ジョイント角プロット
        ax1.plot(time_series, base_joint_data, color = 'blue', label = 'current joint value')
        ax2.plot(time_series, shoulder_joint_data, color = 'blue', label = 'current joint value')
        ax3.plot(time_series, elbow_joint_data, color = 'blue', label = 'current joint value')
        ax4.plot(time_series, wrist1_joint_data, color = 'blue', label = 'current joint value')
        ax5.plot(time_series, wrist2_joint_data, color = 'blue', label = 'current joint value')
        ax6.plot(time_series, wrist3_joint_data, color = 'blue', label = 'current joint value')
        
        # #目標角度をプロットscrewdriver
        ax1.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        ax2.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        ax3.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        ax4.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        ax5.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        ax6.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        
            #x軸のラベル
        ax1.set_xlabel('Time')
        ax2.set_xlabel('Time')
        ax3.set_xlabel('Time')
        ax4.set_xlabel('Time')
        ax5.set_xlabel('Time')
        ax6.set_xlabel('Time')
        
        #y軸のラベル
        ax1.set_ylabel('base joint value[rad/s]')
        ax2.set_ylabel('shoulder joint value[rad/s]')
        ax3.set_ylabel('elbow joint value[rad/s]')
        ax4.set_ylabel('wrist1 joint value[rad/s]')
        ax5.set_ylabel('wrist2 joint value[rad/s]')
        ax6.set_ylabel('wrist3 joint value[rad/s]')
        
        #凡例表示
        ax1.legend(ncol = 2, loc = 'lower right')
        ax2.legend(ncol = 2, loc = 'lower right')
        ax3.legend(ncol = 2, loc = 'lower right')
        ax4.legend(ncol = 2, loc = 'lower right')
        ax5.legend(ncol = 2, loc = 'lower right')
        ax6.legend(ncol = 2, loc = 'lower right')
        
        #重なり解消
        plt.tight_layout()
        
        ax1.grid()
        ax2.grid()
        ax3.grid()
        ax4.grid()
        ax5.grid()
        ax6.grid()
        
        #縦軸横軸の最大値、最小値、目盛り自動設定のグラフ保存
        fig.savefig('./servo_data/joint_vel_values.png')
        
        ###あとでデータの範囲によっては追加するかも
        # #縦軸スケールcdrewdriver
        # base_axis_array = np.arange(1.6, 2.0, 0.05)
        # shoulder_axis_array = np.arange(-1.5, -1.1, 0.05)
        # elbow_axis_array = np.arange(1.5, 1.9, 0.05)
        # wrist1_axis_array = np.arange(-3.7, -3.3, 0.05)
        # wrist2_axis_array = np.arange(-1.8, -1.4, 0.05)
        # wrist3_axis_array = np.arange(-0.2, 0.2, 0.05) 
        
        # #グラフの目盛りのスケールを合わせる
        # ax1.set_yticks(base_axis_array)
        # ax2.set_yticks(shoulder_axis_array)
        # ax3.set_yticks(elbow_axis_array)
        # ax4.set_yticks(wrist1_axis_array)
        # ax5.set_yticks(wrist2_axis_array)
        # ax6.set_yticks(wrist3_axis_array)
        
        # #スケールを合わせたグラフを保存
        # fig.savefig('./servo_data/joint_values_double_scaled.png')
        
    rospy.spin()
    
if __name__ == '__main__':
    try:
        # signal.signal(signal.SIGINT, handler)------------------------
        rospy.init_node('servo_node')
        rospy.loginfo('servo node started')
        # key_command = input()
        # key_command = int(key_command)
        readpickle()
        gen_dsrimvec()
        start_time = time.time()
        vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        vel_input = Float64MultiArray()
        vel_input.layout.data_offset = 1
        main()
    except rospy.ROSInterruptException:
        pass
    