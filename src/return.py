#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import numpy as np 
import matplotlib.animation as animation
import time
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # initialization
    rospy.init_node('returner')

    # 1秒あたりの送信回数
    rate = rospy.Rate(10)

    # define publisher
    joint1_pub = rospy.Publisher('/tilt1_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/tilt2_controller/command', Float64, queue_size=10)
    joint3_pub = rospy.Publisher('/tilt3_controller/command', Float64, queue_size=10)
    joint4_pub = rospy.Publisher('/tilt4_controller/command', Float64, queue_size=10)
    joint5_pub = rospy.Publisher('/tilt5_controller/command', Float64, queue_size=10)

    joint1_pub.publish(0.0)
    joint2_pub.publish(pi/2 - 1.8)
    joint3_pub.publish(0.5)
    joint4_pub.publish(-0.5)
    joint5_pub.publish(0.5)

    while not rospy.is_shutdown():
        #初期位置に戻す
        joint1_pub.publish(0.0)
        joint2_pub.publish(pi/2 - 1.536922657419879)
        joint3_pub.publish(-0.7235753483252765)
        joint4_pub.publish(-0.013347309094605464)
        #joint2_pub.publish(pi/2 - 1.2)
        #joint3_pub.publish(-0.7)
        #joint4_pub.publish(-0.1)
        #joint2_pub.publish(pi/2 - 1.4)
        #joint3_pub.publish(0.4)
        #joint4_pub.publish(0.2)
        #joint2_pub.publish(-0.542007)
        #joint3_pub.publish(-0.659612)
        #joint4_pub.publish(0.388608)
        #joint2_pub.publish(0.4)
        #joint3_pub.publish(0.5)
        #joint4_pub.publish(-0.388608)
        joint5_pub.publish(0.5)
        rate.sleep()