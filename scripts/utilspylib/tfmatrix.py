import math
import numpy as np
import PyKDL
import tf


class TfMatrix(object):

    def __init__(self, base=PyKDL.Frame(), size=np.array([1, 1, 1]), size_m=np.array([0.05, 0.05, 0.05])):
        self.size = size
        self.size_n = size_m
        self.base = base
        self.names = []
        self.transforms = []
        self.frames_map = {}
        self.frames_names = []
        self.frame_index = -1

    def resetIndex(self):
        self.frame_index = -1

    def pickNextFrame(self):
        self.frame_index += 1
        if self.frame_index >= len(self.frames_names):
            raise IndexError('Frames names is over!')
        name = self.frames_names[self.frame_index]
        return (
            name,
            self.frames_map[name]
        )


class TfMatrixCube(TfMatrix):

    def __init__(self, base=PyKDL.Frame(), size=np.array([1, 1, 1]), size_m=np.array([0.05, 0.05, 0.05]), relative_frames=True, execution_order=[0, 1, 2]):
        super(TfMatrixCube, self).__init__(base=base, size=size, size_m=size_m)

        a_lb = range(0, self.size[0])
        b_lb = range(0, self.size[1])
        c_lb = range(0, self.size[2])

        n = self.size[0] * self.size[1] * self.size[2]
        c_list = c_lb
        a_list = []
        b_list = []

        for b in range(0, self.size[1] * self.size[2]):
            a_list.extend(a_lb)
            a_lb.reverse()

        for c in range(0, self.size[2]):
            b_list.extend(b_lb)
            b_lb.reverse()

        indices_list = []
        for s in range(0, n):
            a = s
            b = int(s / self.size[0])
            c = int(s / (self.size[0] * self.size[1]))

            index = (
                a_list[a],
                b_list[b],
                c_list[c]
            )
            indices_list.append(index)

        for index in indices_list:
            x = index[execution_order[0]]
            y = index[execution_order[1]]
            z = index[execution_order[2]]
            dx = size_m[0] * x
            dy = size_m[1] * y
            dz = size_m[2] * z

            current_name = "matrix_cube_{}_{}_{}".format(x, y, z)

            if relative_frames:
                current_frame = self.base * PyKDL.Frame(PyKDL.Vector(dx, dy, dz))
            else:
                current_frame = self.base + PyKDL.Frame(PyKDL.Vector(dx, dy, dz))

            self.frames_names.append(current_name)
            self.frames_map[current_name] = current_frame
