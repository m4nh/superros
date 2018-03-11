#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from superros.comm import RosNode
import PyKDL

node = RosNode("testing_node")
node.setHz(node.setupParameter("hz", 30))

while node.isActive():
    frame = node.retrieveTransform("base_link", "odom")
    if frame is not None:
        t = PyKDL.Frame(
            PyKDL.Rotation.RotZ(1.57),
            PyKDL.Vector(0, 0, 0.5)
        )
        frame = frame * t
        node.broadcastTransform(
            frame, "base_link_2", "odom", node.getCurrentTime()
        )
    node.tick()
