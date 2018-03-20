#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from superros.comm import RosNode
from superros.cameras import CameraRGB
import cv2

node = RosNode("example_rgbcamera")
node.setHz(node.setupParameter("hz", 30))
rgb_topic = node.setupParameter("camera_topic","")

# new frame callback
def newFrame(frame):
    cv2.imshow("image",frame.rgb_image)
    cv2.waitKey(1)

# camera Object
camera = CameraRGB(
    node,
    rgb_topic=rgb_topic,
    compressed_image='compressed'in rgb_topic
)
camera.registerUserCallabck(newFrame)

while node.isActive():
    node.tick()
