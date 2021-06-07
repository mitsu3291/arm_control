#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64

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

    #初期位置に戻す
    while not rospy.is_shutdown():
        joint1_pub.publish(0.0)
        joint2_pub.publish(pi/2 - 1.8)
        joint3_pub.publish(0.5)
        joint4_pub.publish(-0.5)
        joint5_pub.publish(0.5)
        rate.sleep()