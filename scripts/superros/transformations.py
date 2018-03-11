#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from __future__ import division, print_function
import math
import numpy as np
import PyKDL
import rospy
import tf
import cv2
from logger import Logger
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0


def tfToKDL(tf):
    frame = PyKDL.Frame()
    frame.p.x(tf[0][0])
    frame.p.y(tf[0][1])
    frame.p.z(tf[0][2])

    frame.M = PyKDL.Rotation.Quaternion(
        tf[1][0],
        tf[1][1],
        tf[1][2],
        tf[1][3]
    )
    return frame


def tfToPose(tf):
    pose = Pose()
    pose.position.x = tf[0][0]
    pose.position.y = tf[0][1]
    pose.position.z = tf[0][2]
    pose.orientation.x = tf[1][0]
    pose.orientation.y = tf[1][1]
    pose.orientation.z = tf[1][2]
    pose.orientation.w = tf[1][3]
    return pose


def KDLToPose(frame):
    tf = KDLtoTf(frame)
    pose = Pose()
    pose.position.x = tf[0][0]
    pose.position.y = tf[0][1]
    pose.position.z = tf[0][2]
    pose.orientation.x = tf[1][0]
    pose.orientation.y = tf[1][1]
    pose.orientation.z = tf[1][2]
    pose.orientation.w = tf[1][3]
    return pose


def PoseToKDL(pose):
    frame = PyKDL.Frame()
    frame.p = PyKDL.Vector(
        pose.position.x,
        pose.position.y,
        pose.position.z
    )
    frame.M = PyKDL.Rotation.Quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    return frame


def KDLtoTf(frame):
    tf = [
        [frame.p.x(), frame.p.y(), frame.p.z()],
        frame.M.GetQuaternion()
    ]
    return tf


def cvToKDL(Rvec, Tvec):
    """ Converts the OpenCV couple to PyKDL Frame """
    rot, _ = cv2.Rodrigues(Rvec)
    frame = PyKDL.Frame()
    frame.M = PyKDL.Rotation(
        rot[0, 0], rot[0, 1], rot[0, 2],
        rot[1, 0], rot[1, 1], rot[1, 2],
        rot[2, 0], rot[2, 1], rot[2, 2]
    )
    frame.p = PyKDL.Vector(
        Tvec[0],
        Tvec[1],
        Tvec[2]
    )
    return frame


def KDLtoNumpyVector(frame):
    p = frame.p
    q = frame.M.GetQuaternion()
    return np.array([
        p.x(), p.y(), p.z(), q[0], q[1], q[2], q[3]
    ]).reshape(1, 7)


def KLDtoNumpyMatrix(frame):
    M = frame.M
    R = np.array([
        [M[0, 0], M[0, 1], M[0, 2]],
        [M[1, 0], M[1, 1], M[1, 2]],
        [M[2, 0], M[2, 1], M[2, 2]],
    ])
    P = np.transpose(
        np.array([
            frame.p.x(),
            frame.p.y(),
            frame.p.z()
        ])
    )
    P = P.reshape(3, 1)
    T = np.concatenate([R, P], 1)
    T = np.concatenate([T, np.array([0, 0, 0, 1]).reshape(1, 4)], 0)
    return T


def KLDFrameToNumpyRotation(frame):
    M = frame.M
    R = np.array([
        [M[0, 0], M[0, 1], M[0, 2]],
        [M[1, 0], M[1, 1], M[1, 2]],
        [M[2, 0], M[2, 1], M[2, 2]],
    ])
    return R


def KLDFrameToNumpyPosition(frame):
    P = np.transpose(
        np.array([
            frame.p.x(),
            frame.p.y(),
            frame.p.z()
        ])
    )
    return P


def NumpyMatrixToKDL(matrix):
    frame = PyKDL.Frame()
    for i in range(0, 3):
        for j in range(0, 3):
            frame.M[i, j] = matrix[i, j]
    frame.p = PyKDL.Vector(
        matrix[0, 3],
        matrix[1, 3],
        matrix[2, 3]
    )
    return frame


def buildProjectionMatrix(camera_pose, camera_matrix):
    T = camera_pose.Inverse()
    T_sub = KLDtoNumpyMatrix(T)[0:3, :]
    proj_matrix = np.matmul(camera_matrix, T_sub)
    return proj_matrix


def NumpyVectorToKDLVector(array):
    return PyKDL.Vector(
        array[0],
        array[1],
        array[2]
    )


def NumpyVectorToKDL(array):
    frame = PyKDL.Frame()
    frame.p = PyKDL.Vector(
        array[0],
        array[1],
        array[2]
    )
    quaternion = array[3:7]
    quaternion = quaternion / np.linalg.norm(quaternion)
    frame.M = PyKDL.Rotation.Quaternion(
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3]
    )
    return frame


def FrameVectorFromKDL(frame):
    frame_q = frame.M.GetQuaternion()
    frame_v = [frame.p[0],
               frame.p[1],
               frame.p[2],
               frame_q[0],
               frame_q[1],
               frame_q[2],
               frame_q[3], ]
    return frame_v


def FrameVectorToKDL(frame_v):
    frame = PyKDL.Frame(PyKDL.Vector(frame_v[0], frame_v[1], frame_v[2]))
    q = np.array([frame_v[3], frame_v[4], frame_v[5], frame_v[6]])
    q = q / np.linalg.norm(q)
    frame.M = PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3])
    return frame


def TwistToKDLVector(msg):
    x = msg.linear.x
    y = msg.linear.y
    z = msg.linear.z
    return PyKDL.Vector(x, y, z)


def TwistFormKDLVector(vector):
    twist_msg = Twist()
    twist_msg.linear.x = vector.x()
    twist_msg.linear.y = vector.y()
    twist_msg.linear.z = vector.z()
    return twist_msg


def ListToKDLVector(list):
    return PyKDL.Vector(list[0], list[1], list[2])


def KDLVectorToList(vec):
    return [vec.x(), vec.y(), vec.z()]


def KDLFrameToList(frame):
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T = [0, 0, 0]

    for i in range(3):
        for j in range(3):
            R[i][j] = frame.M[i, j]
        T[i] = frame.p[i]
    return [R, T]


def KDLFromArray(chunks, fmt='RPY'):
    if fmt == 'RPY':
        frame = PyKDL.Frame()
        frame.p = PyKDL.Vector(
            chunks[0], chunks[1], chunks[2]
        )
        frame.M = PyKDL.Rotation.RPY(
            chunks[3],
            chunks[4],
            chunks[5]
        )
    if fmt == 'XYZQ':
        frame = PyKDL.Frame()
        frame.p = PyKDL.Vector(
            chunks[0], chunks[1], chunks[2]
        )
        q = np.array([chunks[3],
                      chunks[4],
                      chunks[5],
                      chunks[6]])
        q = q / np.linalg.norm(q)
        frame.M = PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3])
    return frame


def KDLFromString(str, delimiter=' ', fmt='RPY'):

    if fmt != 'RPY':
        Logger.error("Format {} not supported yet!".format(fmt))
        return PyKDL.Frame()

    chunks = map(float, str.split(delimiter))
    frame = PyKDL.Frame()
    frame.p = PyKDL.Vector(
        chunks[0], chunks[1], chunks[2]
    )
    frame.M = PyKDL.Rotation.RPY(
        chunks[3],
        chunks[4],
        chunks[5]
    )
    return frame


def KDLToCv(frame):
    """ Converts the OpenCV couple to PyKDL Frame """
    rot = np.array([
        [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
        [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
        [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]
    ]
    )
    Rvec, _ = cv2.Rodrigues(rot)
    Tvec = np.array([frame.p.x(), frame.p.y(), frame.p.z()])

    return Rvec, Tvec


def KDLFrom2DRF(vx, vy, center):
    rot = PyKDL.Rotation(
        vx[0], vy[0], 0.0,
        vx[1], vy[1], 0.0,
        0, 0, -1
    )
    frame = PyKDL.Frame()
    frame.M = rot
    frame.p.x(center[0])
    frame.p.y(center[1])
    return frame


def KDLFrom2DRFAsArray(arr):
    """ arr = [xx xy cx cy] """
    rf = PyKDL.Frame()
    rf.M = PyKDL.Rotation(
        arr[0], -arr[1], 0,
        arr[1], arr[0], 0,
        0, 0, 1
    )
    rf.p = PyKDL.Vector(arr[2], arr[3], 0)
    return rf


def KDLFromRPY(roll, pitch, yaw):
    frame = PyKDL.Frame()
    frame.M = PyKDL.Rotation.RPY(roll, pitch, yaw)
    return frame


#########################################################################
#########################################################################
#########################################################################
#########################################################################


def KDLVectorToNumpyArray(vector):
    """ Transform a KDL vector to a Numpy Array """
    return np.array(
        [vector.x(), vector.y(), vector.z()]
    ).reshape(3)


def planeCoefficientsFromFrame(frame):
    """ Builds 3D Plane coefficients centered in a Reference Frame """
    normal = frame.M.UnitZ()
    a = normal.x()
    b = normal.y()
    c = normal.z()
    d = -(a * frame.p.x() + b * frame.p.y() + c * frame.p.z())
    return np.array([a, b, c, d]).reshape(4)


def retrieveTransform(listener, base_tf, target_tf, time=rospy.Time(0), print_error=False, none_error=False):
    try:
        tf_transform = listener.lookupTransform(base_tf, target_tf, time)
        frame = tfToKDL(tf_transform)
        return frame
    except (tf.ExtrapolationException, tf.LookupException, tf.ConnectivityException) as e:
        if print_error == True:
            Logger.error("{}".format(str(e)))
        if none_error:
            return None
        else:
            return PyKDL.Frame()


def broadcastTransform(br, frame, frame_id, parent_frame, time=rospy.Time(0)):
    br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()),
                     frame.M.GetQuaternion(),
                     time,
                     frame_id,
                     parent_frame)


def cloneFrame(frame):
    f2 = PyKDL.Frame()
    f2.p = frame.p
    f2.M = frame.M
    return f2


def skew(w):
    R = np.zeros((3, 3))
    R[0, 1] = -w[2]
    R[0, 2] = w[1]
    R[1, 2] = -w[0]

    R[1, 0] = w[2]
    R[1, 0] = -w[1]
    R[2, 1] = w[0]

    return R


def rot2D(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])


def frameDistance(frame1, frame2):
    """ Distance between two frames. [translational_distance, quaternion_distance] """
    dist_t = frame1.p - frame2.p
    q1 = np.array(frame1.M.GetQuaternion())
    q2 = np.array(frame2.M.GetQuaternion())
    dist_a = q1[0] * q2[1:4] - q2[0] * q1[1:4] - skew(q2[1:4]) * q1[1:4]
    return [dist_t, dist_a]


def vector_norm(data, axis=None, out=None):
    """Return length, i.e. Euclidean norm, of ndarray along axis."""

    data = np.array(data, dtype=np.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return math.sqrt(np.dot(data, data))
        data *= data
        out = np.atleast_1d(np.sum(data, axis=axis))
        np.sqrt(out, out)
        return out
    else:
        data *= data
        np.sum(data, axis=axis, out=out)
        np.sqrt(out, out)


def quaternion_about_axis(angle, axis):
    """Return quaternion for rotation about axis.

    >>> q = quaternion_about_axis(0.123, [1, 0, 0])
    >>> np.allclose(q, [0.99810947, 0.06146124, 0, 0])
    True

    """
    qx = axis[0] * math.sin(angle / 2)
    qy = axis[1] * math.sin(angle / 2)
    qz = axis[2] * math.sin(angle / 2)
    qw = math.cos(angle / 2)
    return np.array([qx, qy, qz, qw])


def quaternion_inverse(quaternion):
    """Return inverse of quaternion."""
    q = np.array(quaternion, dtype=np.float64, copy=True)
    np.negative(q[1:], q[1:])
    return q / np.dot(q, q)


def quaternion_angle(quaternion):
    return math.acos(quaternion[3]) * 2.0


def quaternion_multiply(quaternion1, quaternion0):
    """Return multiplication of two quaternions."""
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)


def perpendicular_vector(v):
    """ Finds an arbitrary perpendicular vector to * v * ."""
    # x = y = z = 0 is not an acceptable solution
    if v.x() == v.y() == v.z() == 0:
        raise ValueError('zero-vector')

    if v.x() == 0:
        return PyKDL.Vector(1, 0, 0)
    if v.y() == 0:
        return PyKDL.Vector(0, 1, 0)
    if v.z() == 0:
        return PyKDL.Vector(0, 0, 1)

    # arbitrarily set a = b = 1
    # then the equation simplifies to
    #     c = -(x + y)/z
    return PyKDL.Vector(1, 1, -1.0 * (v.x() + v.y()) / v.z())
