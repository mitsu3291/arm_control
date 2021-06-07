#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np 
import time

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('test_listener')
    sub = rospy.Subscriber('test', String, callback)
    rate = rospy.Rate(10) # 10hz
    for i in range(10):
        rospy.loginfo(i)
        rate.sleep() 
        time.sleep(0.1)

if __name__ == '__main__':
    listener()