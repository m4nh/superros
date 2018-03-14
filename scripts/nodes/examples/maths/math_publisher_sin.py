#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from superros.comm import RosNode
from std_msgs.msg import Float32
import math

node = RosNode("math_publisher_sin")
node.setHz(node.setupParameter("hz", 30))

topic_name = node.setupParameter("topic_name", "math_publisher_sin")
frequency = node.setupParameter("frequency", 1.0)

node.createPublisher(topic_name, Float32)

while node.isActive():
    data = Float32(
        math.sin(2*math.pi*frequency*node.getCurrentTimeInSecs())
    )
    node.getPublisher(topic_name).publish(data)
    node.tick()
