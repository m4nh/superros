#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from superros.comm import RosNode
from std_msgs.msg import Float32

node = RosNode("amplifier")
node.setHz(node.setupParameter("hz", 30))
node.createBufferedSubscriber("in", Float32)
node.createPublisher("out", Float32)

while node.isActive():
    data = Float32(node.getBufferedData("in").data * 2.0)
    node.getPublisher("out").publish(data)
    node.tick()
