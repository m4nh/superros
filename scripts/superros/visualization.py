#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import PyKDL as PyKDL
import random
import numpy as np


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
            return Color(color[0], color[1], color[2], 1.0)
        else:
            index = index % len(Color.PALETTE)
            color = Color.PALETTE[index]
            return Color(color[0], color[1], color[2], 1.0)


def createCube(frame_id, transform=PyKDL.Frame(), size=[1, 1, 1], color=Color(1, 0, 0, 0.8), name=""):
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
    return marker


def createSphere(frame_id, transform=PyKDL.Frame(), radius=0.1, color=Color(1, 0, 0, 0.8), name=""):
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
    return marker


def createLineList(frame_id,  thickness=0.01, points=[], color=Color(1, 0, 0, 0.8), name=""):
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
    return marker


def createArrow(frame_id, transform=PyKDL.Frame(), length=-0.05, shaft=0.005, head=0.009, color=Color(1, 0, 0, 0.8), name="", ap1=np.zeros(3), ap2=np.zeros(3)):
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
    return marker


def createMesh(frame_id, mesh_path='', transform=PyKDL.Frame(), color=Color(), scale=np.array([1, 1, 1]), name=""):
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

    return marker
