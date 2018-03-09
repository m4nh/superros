#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
import numpy as np
import PyKDL

from superros.comm import RosNode
from superros.logger import Logger
from std_msgs.msg import Float32


def topicCallback(data):
    Logger.error(data)
    Logger.warning(data)


node = RosNode("node_test")
node.getName()
node.setupParameter("hz", 30)  #
node.setHz(node.getParameter("hz"))

sin_pub = node.createPublisher("~sin", Float32)
sin_sub = node.createSubscriber("~sin", Float32, topicCallback)

while node.isActive():
    Logger.log(node.getElapsedTimeInSecs())

    sin_val = Float32()
    sin_val.data = math.sin(node.getElapsedTimeInSecs() * 2 * math.pi * 1.0)
    sin_pub.publish(sin_val)

    node.tick()
