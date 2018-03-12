#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from superros.comm import RosNode
from visualization_msgs.msg import MarkerArray
from superros.draw import VisualizationScene, Color
import PyKDL
import math

node = RosNode("draw_test")
node.setHz(node.setupParameter("hz", 30))

pub = node.createPublisher("test", MarkerArray)

# Visualization Scene
scene = VisualizationScene(node, "test")
scene.createCone(name="cone1", color=Color.pickFromPalette(0))
scene.createCone(name="cone2", color=Color.pickFromPalette(1))
scene.createCube(
    name="cube1",
    color=Color.pickFromPalette(2),
    transform=PyKDL.Frame(PyKDL.Vector(-2.0, 0, 0))
)

# Cone2 Frame
cone_2_frame = PyKDL.Frame(PyKDL.Vector(2.0, 0, 0))

while node.isActive():

    # Update Cone2 Pose getting stored Object
    cone_2_frame.M.DoRotZ(0.02)
    scene.getObjectByName("cone2").setTransform(cone_2_frame)

    # Update Cone1 Pose overwriting old Object
    scene.createCone(name="cone1", transform=PyKDL.Frame(
        PyKDL.Vector(
                     0.0,
                     0.0,
                     math.sin(1.0+2*math.pi*1.0*node.getCurrentTimeInSecs())
                     )
    ))

    # Publish World Frame
    node.broadcastTransform(PyKDL.Frame(), "world",
                            "base", node.getCurrentTime())

    # Update Scene
    scene.show()
    # Node Forward
    node.tick()
