#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

current_value = 0.0


def newData(msg):
    global current_value
    current_value = msg.data


rospy.init_node('amplifier', anonymous=True)
rate = rospy.Rate(rospy.get_param("~hz", 30))
sub = rospy.Subscriber("in", Float32, newData)
pub = rospy.Publisher("out", Float32)

while not rospy.is_shutdown():
    data = Float32(current_value*2.0)
    pub.publish(data)
    rate.sleep()
