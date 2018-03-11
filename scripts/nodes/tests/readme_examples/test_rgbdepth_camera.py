#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math
import cv2
import numpy as np
from superros.comm import RosNode
from superros.cameras import CameraRGBD

# Node Creation
node = RosNode("test_rgbdepth_camera")
node.setHz(node.setupParameter("hz", 30))


def imageCallback(frame):
    """Callback called every new FrameRGBD. Each frame contains both
    an RGB image (Color 8UC3) along with a Depth image (Gray 16UC1)

    Arguments:
        frame {FrameRGBD} -- A Frame with RGB+Depth information
    """

    # Converts RGB image to Grayscale and float (values between 0.0 and 1.0)
    gray = cv2.cvtColor(frame.rgb_image, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(float)/255.0

    # Converts Depth image for visualization purposes (values ~ between 0.0 and 1.0)
    depth = frame.depth_image.astype(float)/8000.0

    # Stacks images horizontally
    whole = np.hstack((gray, depth))

    # Show images
    cv2.imshow("rgb+depth image", whole)
    cv2.waitKey(1)


# CameraRGBD object
camera = CameraRGBD(
    rgb_topic='/xtion/rgb/image_raw',
    depth_topic='/xtion/depth/image_raw',
    image_callbacks=imageCallback,
    approx_time_sync=0.5  # this is the time tollerance for sync
)

node.await()
