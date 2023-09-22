#!/usr/bin/env python
#coding: UTF-8

import rospy

from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray
from moveit_tutorials.srv import GetCurrentJointVel, GetCurrentJointVelResponse

###image_rawを常にsubscribeし、client(vel_servo_v2.py)からのサービスコールがあったときにjoint statesを返す
vel = []
    
#subsciberコールバック関数内では変数に取得したデータを代入
def get_joint_state(data):
    global vel
    vel = data.velocity
    # print('vel')
    return vel

#サービスコールのコールバックの中で変数(vel)を送信
def return_vel(res):
    global vel
    # print(vel)
    # print(type(vel)) ###tupleでした,()で出力されました
    print('sent response')
    return GetCurrentJointVelResponse(vel)

def main():
	rospy.loginfo('ready to serve')
	rospy.Subscriber('/joint_states', JointState, get_joint_state)
	rospy.Service('getvel', GetCurrentJointVel, return_vel)
	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('capture_server')
		main()
	except rospy.ROSInterruptException:
		pass
