#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np 

def talker():
    rospy.init_node('test_talker')
    pub = rospy.Publisher('test', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ran = np.random.rand()
        rospy.loginfo(ran)
        pub.publish(ran)
        rate.sleep()

if __name__ == '__main__':
    talker()