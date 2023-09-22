#!/usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import cv2
import moveit_commander
import sys
import pandas as pd
import math
from time import sleep
import matplotlib.pyplot as plt

import csv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

pinv_int_mat_double = np.empty((6,2073600))
move_group = None
I_dsr_vec = np.empty((2073600, 1))
rmse = 100
lmbd = 0.2
t = 0.03 
nop = 2073600 #number of pixels
rmse_data = [] 
new_joint_goal_data = [] 
# pose_data = []

base_joint_data = []
shoulder_joint_data = []
elbow_joint_data = []
wrist1_joint_data = []
wrist2_joint_data = []
wrist3_joint_data = []

# desired_pose = [-0.10968, 0.372502, 0.236982]
desired_pose = [-0.1558, 0.3379, 0.3698] #screwdriver
pose_dist_data = []

def readpickle():
    global pinv_int_mat_double
    pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat_double.pickle')
    pinv_int_mat_double = pickledata.values
    return pinv_int_mat_double

def gen_dsrimvec(): 
    global I_dsr_vec
    I_dsr = cv2.imread('./input_dsrim/kensyo_desired_image.png')
    I_dsr_gry = cv2.cvtColor(I_dsr, cv2.COLOR_BGR2GRAY)
    I_dsr_arr = np.array(I_dsr_gry, dtype = 'float64')
    I_dsr_vec = I_dsr_arr.reshape(-1,1)
    return I_dsr_vec

def moveit_settings():
    global move_group
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    move_group.set_max_velocity_scaling_factor(value=0.1)
    move_group.set_max_acceleration_scaling_factor(value=0.1)
    print('moveit settings done')
    return move_group

def servo():
    global pinv_int_mat_double
    global move_group
    global lmbd
    global I_dsr_vec
    global nop
    global rmse
    global rmse_data
    global new_joint_goal_data
    # global pose_data
    
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    
    global desired_pose
    global pose_dist_data
    
    #subscribe image raw with rospy.wait_for_message
    # print('wait')
    image_raw = rospy.wait_for_message('/camera/color/image_raw', Image)
    # print('received')
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8') ###この時点でnumpy.arrayになってる1080,1920,3
    # image_name = './servo_data/loop_image/' + str(i) + '_loop_image.png'*****************************
    # cv2.imwrite(image_name, bgr)*****************************
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
    d_vel = lmbd*(np.dot(pinv_int_mat_double, dI)) 
    # print(d_vel)
    
    #位置変化分計算
    d_joint_values = t*d_vel
    # print(d_joint_values) 
    
    #set_joint_valuesに代入できる形に変換
    arr_d_joint_values = np.empty((6))
    arr_d_joint_values = d_joint_values[:,0]
    
    #現在ジョイント角に位置変化分を足す
    ###初期化的なやつ必要？？？movegroup.clear joint values みたいなのあった気がする
    current_joint_values = np.array(move_group.get_current_joint_values())
    # print(current_joint_values.shape) #(6,)の形になってた
    new_joint_goal = current_joint_values + arr_d_joint_values
    print(new_joint_goal)
    new_joint_goal_list = new_joint_goal.tolist()
    new_joint_goal_data.append(new_joint_goal_list)
    
    posex = move_group.get_current_pose().pose.position.x
    posey = move_group.get_current_pose().pose.position.y
    posez = move_group.get_current_pose().pose.position.z
    # pose_data.append([posex, posey, posez])
    pose_dist = 1000*math.sqrt((desired_pose[0] - posex)**2 + (desired_pose[1] - posey)**2 + (desired_pose[2] - posez)**2)
    pose_dist_data.append(pose_dist)
    
    #動かす
    move_group.set_joint_value_target(new_joint_goal)
    move_group.go(wait=True)
    move_group.stop() 
    move_group.forget_joint_values('manipulator') ###保存された関節角を消去します。
    sleep(0.5)
    
    rmse_data.append(rmse)
    
    # #各ジョイント角グラフのy軸とx軸
    base_joint_data.append(new_joint_goal[0])
    shoulder_joint_data.append(new_joint_goal[1])
    elbow_joint_data.append(new_joint_goal[2])
    wrist1_joint_data.append(new_joint_goal[3])
    wrist2_joint_data.append(new_joint_goal[4])
    wrist3_joint_data.append(new_joint_goal[5])
    # return rmse, rmse_data
    return rmse
    # return rmse, rmse_data, new_joint_goal_data
    
def main():
    i = 1
    filename = './servo_data/loop_joint_goal_values_double.csv'
    filename2 = './servo_data/loop_rmse_data.csv'
    filename3 = './servo_data/loop_dist_data.csv'
    loop = 1000
    with open (filename, 'w') as f, open(filename2, 'w') as f2, open(filename3, 'w') as f3:
    # with open (filename, 'w') as f, open(filename2, 'w') as f2:
        for i in range(loop):
            global rmse
            if rmse > 10:
                print('loop %d' % i)
                i += 1
                servo()
            else:
                print('done')
                break
        writer = csv.writer(f)
        writer.writerows(new_joint_goal_data)
        writer2 = csv.writer(f2)
        writer2.writerow(rmse_data)
        writer3 = csv.writer(f3)
        writer3.writerow(pose_dist_data)
        x = range(len(base_joint_data))
        
        #6つのグラフの配置
        fig = plt.figure(figsize = (10,10))
        ax1 = fig.add_subplot(3,2,1)
        ax2 = fig.add_subplot(3,2,2)
        ax3 = fig.add_subplot(3,2,3)
        ax4 = fig.add_subplot(3,2,4)
        ax5 = fig.add_subplot(3,2,5)
        ax6 = fig.add_subplot(3,2,6)
        
        #各ループごとの各ジョイント角プロット
        ax1.plot(x, base_joint_data, color = 'blue', label = 'current joint value')
        ax2.plot(x, shoulder_joint_data, color = 'blue', label = 'current joint value')
        ax3.plot(x, elbow_joint_data, color = 'blue', label = 'current joint value')
        ax4.plot(x, wrist1_joint_data, color = 'blue', label = 'current joint value')
        ax5.plot(x, wrist2_joint_data, color = 'blue', label = 'current joint value')
        ax6.plot(x, wrist3_joint_data, color = 'blue', label = 'current joint value')
        
        # #目標角度をプロットscrewdriver
        ax1.axhline(y = 1.738, color = 'red', label = 'desired joint value')
        ax2.axhline(y = -1.232, color = 'red', label = 'desired joint value')
        ax3.axhline(y = 1.685, color = 'red', label = 'desired joint value')
        ax4.axhline(y = -3.549, color = 'red', label = 'desired joint value')
        ax5.axhline(y = -1.72, color = 'red', label = 'desired joint value')
        ax6.axhline(y = 0.0, color = 'red', label = 'desired joint value')
        
        #x軸のラベル
        ax1.set_xlabel('Loop')
        ax2.set_xlabel('Loop')
        ax3.set_xlabel('Loop')
        ax4.set_xlabel('Loop')
        ax5.set_xlabel('Loop')
        ax6.set_xlabel('Loop')
        
        #y軸のラベル
        ax1.set_ylabel('base joint value[rad]')
        ax2.set_ylabel('shoulder joint value[rad]')
        ax3.set_ylabel('elbow joint value[rad]')
        ax4.set_ylabel('wrist1 joint value[rad]')
        ax5.set_ylabel('wrist2 joint value[rad]')
        ax6.set_ylabel('wrist3 joint value[rad]')
        
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
        fig.savefig('./servo_data/joint_values_double.png')
        
        # #縦軸スケール
        # base_axis_array = np.arange(1.4, 1.8, 0.05)
        # shoulder_axis_array = np.arange(-1.5, -1.1, 0.05)
        # elbow_axis_array = np.arange(1.3, 1.7, 0.05)
        # wrist1_axis_array = np.arange(-2.0, -1.6, 0.05)
        # wrist2_axis_array = np.arange(-1.8, -1.4, 0.05)
        # wrist3_axis_array = np.arange(-0.2, 0.2, 0.05)
        
        #縦軸スケールcdrewdriver
        base_axis_array = np.arange(1.6, 2.0, 0.05)
        shoulder_axis_array = np.arange(-1.5, -1.1, 0.05)
        elbow_axis_array = np.arange(1.5, 1.9, 0.05)
        wrist1_axis_array = np.arange(-3.7, -3.3, 0.05)
        wrist2_axis_array = np.arange(-1.8, -1.4, 0.05)
        wrist3_axis_array = np.arange(-0.2, 0.2, 0.05) 
        
        #グラフの目盛りのスケールを合わせる
        ax1.set_yticks(base_axis_array)
        ax2.set_yticks(shoulder_axis_array)
        ax3.set_yticks(elbow_axis_array)
        ax4.set_yticks(wrist1_axis_array)
        ax5.set_yticks(wrist2_axis_array)
        ax6.set_yticks(wrist3_axis_array)
        
        #スケールを合わせたグラフを保存
        fig.savefig('./servo_data/joint_values_double_scaled.png')
        
        fig_dist = plt.figure()
        plt.xlabel('Loop')
        plt.ylabel('Three-Dimensional Distance[mm]')
        distloop = range(len(pose_dist_data))
        plt.plot(distloop, pose_dist_data, 'b-')
        plt.grid()
        fig_dist.savefig('./servo_data/3dposedist.png')

        #generate graph
        fig_rmse = plt.figure()
        plt.xlabel('Loop')
        plt.ylabel('RMSE')
        rmseloop = range(len(rmse_data))
        plt.plot(rmseloop, rmse_data, 'b-')
        plt.grid()
        fig_rmse.savefig('./servo_data/rmse_double.png')
    
if __name__ == '__main__':
    try:
        # signal.signal(signal.SIGINT, handler)------------------------
        rospy.init_node('servo')
        rospy.loginfo('servo node started')
        readpickle()
        moveit_settings()
        gen_dsrimvec()
        main()
    except rospy.ROSInterruptException:
        pass
    