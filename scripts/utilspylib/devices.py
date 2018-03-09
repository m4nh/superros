from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import PyKDL
import copy
import math
import tf
import numpy as np


class ForceSensor(object):
    def __init__(self, relative_frame=PyKDL.Frame(), translation_mag=0.01, rotation_mag=0.01, trans_relative=True):
        self.relative_frame = relative_frame
        self.translation_mag = translation_mag
        self.rotation_mag = rotation_mag
        self.current_msg = Twist()
        self.trans_relative = trans_relative
        self.transThresholds = [0, 0, 0]

    def setRelativeFrame(self, frame):
        self.relative_frame = frame

    def setTransThreshold(self, th):
        self.transThresholds = th

    def setMagnitude(self, translation_mag, rotation_mag):
        self.translation_mag = translation_mag
        self.rotation_mag = rotation_mag

    def update(self, twist_msg):
        self.current_msg = twist_msg

    def getAxis(self, name):
        if name == 'x':
            if math.fabs(self.current_msg.linear.x) > self.transThresholds[0]:
                return self.current_msg.linear.x
            else:
                return 0
        if name == 'y':
            if math.fabs(self.current_msg.linear.y) > self.transThresholds[1]:
                return self.current_msg.linear.y
            else:
                return 0
        if name == 'z':
            if math.fabs(self.current_msg.linear.z) > self.transThresholds[2]:
                return self.current_msg.linear.z
            else:
                return 0
        else:
            return 0

    def getTTrans(self):
        frame = PyKDL.Frame(PyKDL.Vector(
            self.translation_mag * self.getAxis('x'),
            self.translation_mag * self.getAxis('y'),
            self.translation_mag * self.getAxis('z')))
        return frame

    def getTRot(self):
        frame = PyKDL.Frame()
        return frame

    def transformT(self, frame_to_transform):
        new_frame = frame_to_transform
        # Trans
        new_frame = new_frame * self.getTTrans()
        # Rot
        new_frame = new_frame * self.getTRot()
        return new_frame

    def output(self):
        return self.getTTrans() * self.getTRot()

##################################################################
##################################################################
##################################################################
##################################################################


class Joystick(object):
    KEY_X = 2
    KEY_Y = 3
    KEY_B = 1
    KEY_A = 0
    KEY_BACK = 6
    KEY_START = 7
    KEY_LOG = 8

    def __init__(self, translation_mag=0.01, rotation_mag=0.01, trans_relative=False):
        self.translation_mag = translation_mag
        self.rotation_mag = rotation_mag
        self.current_msg = Joy()
        self.current_msg.axes = [0, 0, 1, 0, 0, 1, 0, 0]
        self.current_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0]
        self.trans_relative = trans_relative
        self.available_keys = [
            Joystick.KEY_A,
            Joystick.KEY_Y,
            Joystick.KEY_B,
            Joystick.KEY_X,
            Joystick.KEY_START,
            Joystick.KEY_BACK,
            Joystick.KEY_LOG
        ]
        self.keyCallback = None
        self.forced_target = None

    def setForcedTarget(self, frame):
        print("Forcing target SET")
        self.forced_target = frame

    def registerCallback(self, callback):
        self.keyCallback = callback

    def updateCallback(self, diff):
        down_keys = []
        up_keys = []
        for key in self.available_keys:
            if diff[key] < 0:
                down_keys.append(key)
            if diff[key] > 0:
                up_keys.append(key)

        if len(down_keys) > 0 or len(up_keys) > 0:
            if self.keyCallback:
                self.keyCallback(down_keys, up_keys)

    def update(self, msg):
        diff = np.array(self.current_msg.buttons) - np.array(msg.buttons)
        self.updateCallback(diff)
        self.current_msg = msg

    def getButtons(self):
        return self.current_msg.buttons

    def getAxes(self):
        return self.current_msg.axes

    def getAxis(self, name):
        if name == 'x':
            return self.getAxes()[1]
        if name == 'y':
            return self.getAxes()[3]
        if name == 'z':
            return 0.5 * (self.getAxes()[2] - 1.0) - 0.5 * (self.getAxes()[5] - 1.0)
        if name == 'pitch':
            return self.getAxes()[7]
        if name == 'roll':
            return self.getAxes()[6]
        if name == 'yaw':
            return -self.getButtons()[4] + self.getButtons()[5]
        else:

            return 0

    def getTTrans(self):
        pp = PyKDL.Vector(
            self.translation_mag * self.getAxis('x'),
            self.translation_mag * self.getAxis('y'),
            self.translation_mag * self.getAxis('z'))
        frame = PyKDL.Frame()
        frame.p = pp

        return frame

    def getTRot(self):
        frame = PyKDL.Frame()
        frame.M = PyKDL.Rotation.RPY(
            self.rotation_mag * self.getAxis('roll'),
            self.rotation_mag * self.getAxis('pitch'),
            self.rotation_mag * self.getAxis('yaw')
        )
        return frame

    def transformT(self, frame_to_transform):
        if self.forced_target != None:
            print("Forced rtarget!")
            frame = PyKDL.Frame()
            frame.p = self.forced_target.p
            frame.m = self.forced_target.M
            self.forced_target = None
            return frame

        new_frame = frame_to_transform
        # Trans
        if self.trans_relative:
            new_frame = new_frame * self.getGlobalT()
        else:
            ff = self.getTTrans()
            new_frame.p = new_frame.p + ff.p

        # Rot
        new_frame = new_frame * self.getTRot()
        return new_frame

    def getGlobalT(self):
        frame = self.getTTrans()
        return frame

    def nearestOrthogonalFrame(self, input_frame):
        rpy = input_frame.M.GetRPY()
        pi_2 = np.pi * 0.5
        nroll = int(rpy[0] / pi_2) * pi_2
        npitch = int(rpy[1] / pi_2) * pi_2
        nyaw = int(rpy[2] / pi_2) * pi_2
        new_frame = PyKDL.Frame()
        new_frame.p = input_frame.p
        new_frame.M = PyKDL.Rotation.RPY(
            nroll, npitch, nyaw
        )
        return new_frame
        # print("Current ", rpy)
        # Logger.log("{0:.2f}, {1:.2f}, {2:.2f}".format(nroll, npitch, nyaw))
