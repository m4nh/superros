#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
import cv2
import numpy as np
from superros.comm import RosNode
from superros.cameras import CameraRGB

# Node Creation
node = RosNode("test_rgb_camera")
node.setHz(node.setupParameter("hz", 30))


def imageCallback(frame):
    """Callback called every new FrameRGB. Contains an RGB image (Color 8UC3) 

    Arguments:
        frame {FrameRGB} -- A Frame with RGB information
    """
    # Show images
    cv2.imshow("rgb image", frame.rgb_image)
    cv2.waitKey(1)


# CameraRGB object
camera = CameraRGB(
    rgb_topic='/xtion/rgb/image_raw',
    image_callbacks=imageCallback
)

node.await()
