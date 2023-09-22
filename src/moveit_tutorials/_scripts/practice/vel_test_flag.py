#!/usr/bin/env python
#coding: UTF-8

import rospy
from std_msgs.msg import Int32
import time

# rospy.init_node('vel_flag_publisher')
# pub = rospy.Publisher('vel_flag', Int32, queue_size = 10)

# rate = rospy.Rate(1)
# flag = 0
# # counter = 0

# while not rospy.is_shutdown():
#     pub.publish(flag)
#     flag += 1
#     # for counter in range(1000):
#     #     counter += 1
#     #     if counter == 1000:
#     #       flag += 1
#     #普通何秒スリープさせるっけ
#     rate.sleep()
    
def flag_publisher():
    count = 0
    while not rospy.is_shutdown():
        if count < 5:  
            if count % 2 == 0:
                start_time = time.time()
                while time.time() - start_time < 3:
                    flag = 1
                    # print(flag)
                    pub.publish(flag)
                    rate.sleep()
            if count % 2 == 1:
                start_time = time.time()
                while time.time() - start_time < 3:
                    flag = 0
                    # print(flag)
                    pub.publish(flag)
                    rate.sleep()
        else:
            flag = 2
            print(flag)
            pub.publish(flag)
            rate.sleep()                                
        count += 1
        print(count)
        
if __name__ == '__main__':
    rospy.init_node('flag_publisher')
    pub = rospy.Publisher('vel_flag', Int32, queue_size = 10)
    rate = rospy.Rate(100)
    try:
        flag_publisher()
    except rospy.ROSInterruptException:
        pass
                


