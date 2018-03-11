#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from superros.comm import RosNode
from std_msgs.msg import Float32
import math

node = RosNode("sin")
node.setHz(node.setupParameter("hz", 30))

node.createPublisher("in", Float32)
node.createPublisher("in2", Float32)
node.createPublisher("in3", Float32)

while node.isActive():
    data = Float32(
        math.sin(2*3.14*1.0*node.getCurrentTimeInSecs())
    )
    data2 = Float32(
        math.sin(2*3.14*2.0*node.getCurrentTimeInSecs())
    )
    data3 = Float32(
        math.sin(2*3.14*3.0*node.getCurrentTimeInSecs())
    )
    node.getPublisher("in").publish(data)
    node.getPublisher("in2").publish(data2)
    node.getPublisher("in3").publish(data3)
    node.tick()
