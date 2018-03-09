#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import math
import numpy as np
import PyKDL

from superros.comm import RosNode
from superros.logger import Logger


if __name__ == '__main__':

    #⬢⬢⬢⬢⬢➤ NODE

    node = RosNode("test_visione")
    node.getName()
    node.setupParameter("hz", 15)  #
    node.setHz(node.getParameter("hz"))

    while node.isActive():
        Logger.log("ok")
        node.tick()
