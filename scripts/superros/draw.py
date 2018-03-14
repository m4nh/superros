#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import transformations
import PyKDL as PyKDL
import random
import numpy as np
import math


class Color:
    PALETTE_COUNTER = 0
    PALETTE = [
        (244, 67, 54),
        (33, 150, 243),
        (76, 175, 80),
        (255, 193, 7),
        (156, 39, 176),
        (96, 125, 139),
        (63, 81, 181),
        (255, 87, 34)
    ]

    def __init__(self, r=1.0, g=1.0, b=1.0, a=1):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    def toOpenCV(self):
        return (self.b, self.g, self.r)

    @staticmethod
    def randomColor(a=1.0):
        return Color(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), a)

    @staticmethod
    def pickFromPalette(index):
        if index < 0:
            color = Color.PALETTE[Color.PALETTE_COUNTER]
            Color.PALETTE_COUNTER += 1
            Color.PALETTE_COUNTER = Color.PALETTE_COUNTER % len(Color.PALETTE)
        else:
            index = index % len(Color.PALETTE)
            color = Color.PALETTE[index]
        return Color(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0)


class VisualizationObject(Marker):

    def __init__(self):
        super(VisualizationObject, self).__init__()

    def setTransform(self, transform):
        self.pose = transformations.KDLToPose(transform)


class VisualizationScene(object):

    def __init__(self, node, topic_name=""):
        self.objects_map = {}
        self._node = node

        self._visualization_topic = None
        if len(topic_name) > 0:
            self._visualization_topic = node.createPublisher(
                topic_name, MarkerArray
            )

    def getObjectByName(self, name):
        if name in self.objects_map:
            return self.objects_map[name]
        return None

    def createCube(self,  name, frame_id='world', transform=PyKDL.Frame(), size=[1, 1, 1], color=Color(1, 0, 0, 0.8)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE
        marker.action = marker.ADD
        marker.ns = name
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        marker.pose.position.x = transform.p.x()
        marker.pose.position.y = transform.p.y()
        marker.pose.position.z = transform.p.z()
        quat = transform.M.GetQuaternion()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        self.objects_map[name] = marker
        return marker

    def createSphere(self, name, frame_id='world', transform=PyKDL.Frame(), radius=0.1, color=Color(1, 0, 0, 0.8)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.SPHERE
        marker.ns = name
        marker.action = marker.ADD
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.pose.position.x = transform.p.x()
        marker.pose.position.y = transform.p.y()
        marker.pose.position.z = transform.p.z()
        quat = transform.M.GetQuaternion()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        self.objects_map[name] = marker
        return marker

    def createLineList(self, name, frame_id='world',  thickness=0.01, points=[], color=Color(1, 0, 0, 0.8)):
        """ Creates a LineList Marker """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.LINE_LIST
        marker.action = marker.ADD
        marker.ns = name
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        marker.scale.x = thickness
        marker.scale.y = thickness
        marker.scale.z = thickness
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            if point.size > 2:
                p.z = point[2]
            else:
                p.z = 0
            marker.points.append(p)

        self.objects_map[name] = marker
        return marker

    def createArrow(self, name, frame_id='world', transform=PyKDL.Frame(), length=-0.05, shaft=0.005, head=0.009, color=Color(1, 0, 0, 0.8),  ap1=np.zeros(3), ap2=np.zeros(3)):
        """ Creates an Arrow Marker """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.ARROW
        marker.action = marker.ADD
        marker.ns = name
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        marker.scale.x = shaft
        marker.scale.y = head
        marker.scale.z = 0
        marker.pose.position.x = transform.p.x()
        marker.pose.position.y = transform.p.y()
        marker.pose.position.z = transform.p.z()
        quat = transform.M.GetQuaternion()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        p1 = Point()
        p2 = Point()
        if length < 0:
            p1.x = ap1[0]
            p1.y = ap1[1]
            p1.z = ap1[2]
            p2.x = ap2[0]
            p2.y = ap2[1]
            p2.z = ap2[2]
        else:
            p1.x = 0.0
            p1.y = 0.0
            p1.z = 0.0
            p2.x = 0.0
            p2.y = 0.0
            p2.z = length
        marker.points.append(p1)
        marker.points.append(p2)

        self.objects_map[name] = marker
        return marker

    def createCone(self, name, frame_id='world', transform=PyKDL.Frame(), scale=[1.0, 1.0, 1.0], color=Color(1, 0, 0, 1.0)):
        marker = VisualizationObject()
        marker.header.frame_id = frame_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = marker.ADD
        marker.ns = name
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        marker.scale.x = 1  # scale[0]
        marker.scale.y = 1  # scale[1]
        marker.scale.z = 1  # scale[2]

        marker.pose = transformations.KDLToPose(transform)

        delta_theta = math.pi / 16.0
        theta = 0.0
        marker.points = []

        for i in range(0, 32):
            p0 = Point()
            p0.x = 0
            p0.y = 0
            p0.z = marker.scale.z

            p1 = Point()
            p1.z = 0.0
            p1.x = marker.scale.x * math.cos(theta)
            p1.y = marker.scale.y * math.sin(theta)

            p2 = Point()
            p2.z = 0.0
            p2.x = marker.scale.x * math.cos(theta + delta_theta)
            p2.y = marker.scale.y * math.sin(theta + delta_theta)

            marker.points.append(p0)
            marker.points.append(p1)
            marker.points.append(p2)

            p3 = Point()
            p3.x = p3.y = p3.z = 0.0
            marker.points.append(p2)
            marker.points.append(p1)
            marker.points.append(p3)

            theta += delta_theta

        self.objects_map[name] = marker
        return marker

    def createMesh(self, name, frame_id='world', mesh_path='', transform=PyKDL.Frame(), color=Color(), scale=np.array([1, 1, 1])):
        """ Creates a Mesh Marker """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE
        marker.ns = name
        marker.action = marker.ADD
        marker.mesh_resource = mesh_path

        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.pose.position.x = transform.p.x()
        marker.pose.position.y = transform.p.y()
        marker.pose.position.z = transform.p.z()
        quat = transform.M.GetQuaternion()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        self.objects_map[name] = marker
        return marker

    def show(self):
        markers = MarkerArray()
        for k, v in self.objects_map.items():
            markers.markers.append(v)
        if self._visualization_topic is not None:
            self._visualization_topic.publish(markers)
