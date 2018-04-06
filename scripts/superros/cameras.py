#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import cv2
import numpy as np
import message_filters
from comm import RosNode
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv_bridge

from logger import Logger
import os
import sys


class FrameRGB(object):
    CV_BRIDGE = CvBridge()

    def __init__(self, rgb_image=None, time=None):
        self.rgb_image = rgb_image
        self.time = time
        if self.rgb_image == None:
            self.valid = False
        else:
            self.valid = True

    def isValid(self):
        return self.valid

    @staticmethod
    def buildFromMessages(rgb_msg):
        frame = FrameRGB()
        frame.time = rgb_msg.header.stamp

        frame.rgb_image = FrameRGBD.CV_BRIDGE.imgmsg_to_cv2(
            rgb_msg, FrameRGB.getEncodingType(rgb_msg.encoding))

        frame.valid = True
        return frame

    @staticmethod
    def getEncodingType(source_encoding):
        if source_encoding == 'rgb8':
            return 'bgr8'

    @staticmethod
    def buildFromMessagesCompressed(rgb_msg):
        frame = FrameRGB()
        frame.time = rgb_msg.header.stamp

        np_arr = np.fromstring(rgb_msg.data, np.uint8)
        try:
            frame.rgb_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        except:
            try:
                frame.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except:
                return frame

        frame.valid = True
        return frame


class FrameRGBD(FrameRGB):

    def __init__(self, rgb_image=None, depth_image=None, time=None):
        super(FrameRGBD, self).__init__(rgb_image=rgb_image, time=time)
        self.depth_image = depth_image
        self.depth_scale = 1
        if self.rgb_image == None or self.depth_image == None:
            self.valid = False
        else:
            self.valid = True

    def isValid(self):
        return self.valid

    @staticmethod
    def buildFromMessages(rgb_msg, depth_msg, depth_scale=1000):
        frame = FrameRGBD()
        frame.depth_scale = 1000
        frame.time = rgb_msg.header.stamp

        frame.rgb_image = FrameRGBD.CV_BRIDGE.imgmsg_to_cv2(
            rgb_msg, FrameRGB.getEncodingType(rgb_msg.encoding))

        try:
            frame.depth_image = FrameRGBD.CV_BRIDGE.imgmsg_to_cv2(
                depth_msg, "16UC1")
        except CvBridgeError as e:
            Logger.error(e)
            return frame

        frame.valid = True
        return frame


class Camera(object):

    def __init__(self, node, configuration_file=None):
        self.node = node
        self.configuration_file = configuration_file
        self.width = 640
        self.height = 480
        self.fx = 570.0
        self.fy = 570.0
        self.cx = 320.0
        self.cy = 240.0
        self.k1 = 0.0
        self.k2 = 0.0
        self.k3 = 0.0
        self.p1 = 0.0
        self.p2 = 0.0

        # Loads configuration from file, if any
        if self.configuration_file is not None:
            self.loadConfigurationFromFile(configuration_file)

        # Builds Camera matrix
        self.camera_matrix = np.array([])
        self.camera_matrix_inv = np.array([])
        self.distortion_coefficients = np.array([])
        self.buildCameraMatrix()

    def loadConfigurationFromFile(self, filename):
        _, ext = os.path.splitext(filename)
        if filename == 'txt':
            return self.loadConfigurationFromFile(filename)
        else:
            Logger.error(
                "Unsupported Configuration File '{}'".format(filename))

    def loadConfigurationFromTXTFile(self, filename):
        raw = np.loadtxt(filename)
        self.width = int(raw[0])
        self.height = int(raw[1])
        self.fx = raw[2]
        self.fy = raw[3]
        self.cx = raw[4]
        self.cy = raw[5]
        self.k1 = raw[6]
        self.k2 = raw[7]
        self.p1 = raw[8]
        self.p2 = raw[9]
        if len(raw) > 10:
            self.k3 = raw[10]
        else:
            self.k3 = 0.0

    def buildCameraMatrix(self):
        self.camera_matrix = np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
        self.distortion_coefficients = np.array([
            self.k1, self.k2, self.p1, self.p2, self.k3
        ])
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)

    def getConfigurationFile(self):
        return self.configuration_file


class CameraRGBD(Camera):

    def __init__(self, node, configuration_file=None, rgb_topic='', depth_topic='', callback_buffer_size=1, image_callbacks=None, approx_time_sync=0.1):
        super(CameraRGBD, self).__init__(
            node, configuration_file=configuration_file)
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.callback_buffer_size = callback_buffer_size
        self.approx_time_sync = approx_time_sync

        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], callback_buffer_size, approx_time_sync)
        ts.registerCallback(self.topicCallback)

        self._user_callbacks = []

        if image_callbacks is not None:
            if not isinstance(image_callbacks, list):
                image_callbacks = [image_callbacks]
            self._user_callbacks = image_callbacks

    def registerUserCallabck(self, callback):
        self._user_callbacks.append(callback)

    def topicCallback(self, rgb_msg, depth_msg):
        frame = FrameRGBD.buildFromMessages(
            rgb_msg, depth_msg)
        for c in self._user_callbacks:
            c(frame)


class CameraRGB(Camera):

    def __init__(self, node, configuration_file=None, rgb_topic='', callback_buffer_size=1, compressed_image=False, image_callbacks=None):
        super(CameraRGB, self).__init__(
            node, configuration_file=configuration_file)
        self.rgb_topic = rgb_topic
        self.callback_buffer_size = callback_buffer_size
        self.compressed_image = compressed_image

        if compressed_image:

            self.rgb_sub = node.createSubscriber(
                self.rgb_topic, CompressedImage, self.topicCallback, queue_size=self.callback_buffer_size)
        else:
            self.rgb_sub = node.createSubscriber(
                self.rgb_topic, Image, self.topicCallback, queue_size=self.callback_buffer_size)
        self._user_callbacks = []

        if image_callbacks is not None:
            if not isinstance(image_callbacks, list):
                image_callbacks = [image_callbacks]
            self._user_callbacks = image_callbacks

    def registerUserCallabck(self, callback):
        self._user_callbacks.append(callback)

    def topicCallback(self, rgb_msg):
        if self.compressed_image:
            frame = FrameRGB.buildFromMessagesCompressed(rgb_msg)
        else:
            frame = FrameRGB.buildFromMessages(rgb_msg)
        for c in self._user_callbacks:
            c(frame)
