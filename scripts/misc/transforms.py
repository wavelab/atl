#!/usr/bin/env python3
import numpy as np
from numpy import dot
from numpy.linalg import inv


class Transform:
    def __init__(self, **kwargs):
        self.R = kwargs["R"]
        self.t = kwargs.get("t", np.array([0.0, 0.0, 0.0]))
        self.T = np.array([
            [self.R[0][0], self.R[0][1], self.R[0][2], self.t[0]],
            [self.R[1][0], self.R[1][1], self.R[1][2], self.t[1]],
            [self.R[2][0], self.R[2][1], self.R[2][2], self.t[2]],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def transform(self, x):
        result = dot(self.T, np.array([x[0], x[1], x[2], 1.0]))
        return result[0:3]

    def inverse_transform(self, x):
        result = dot(inv(self.T), np.array([x[0], x[1], x[2], 1.0]))
        return result[0:3]


if __name__ == "__main__":
    R_nwu_ned = np.array([[1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0],
                          [0.0, 0.0, -1.0]])

    R_ned_nwu = np.array([[1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0],
                          [0.0, 0.0, -1.0]])

    R_nwu_edn = np.array([[0.0, 0.0, 1.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0]])

    R_edn_nwu = np.array([[0.0, -1.0, 0.0],
                          [0.0, 0.0, -1.0],
                          [1.0, 0.0, 0.0]])

    T = Transform(R=R_ned_nwu, t=np.array([1.0, 1.0, 1.0]))
    x_ned = T.transform(np.array([1.0, 1.0, 1.0]))
    x_nwu = T.inverse_transform(x_ned)

    # x_nwu = np.array([1.0, 2.0, 3.0])
    # x_edn = dot(R_edn_nwu, x_nwu)
    # print(x_edn)

    # # NED -> NWU
    # x_ned = np.array([1.0, 2.0, 3.0])
    # x_nwu = dot(R_nwu_ned, x_ned)
    #
    # # NWU -> NED
    # R_ned_nwu = np.array([[1.0, 0.0, 0.0],
    #                       [0.0, -1.0, 0.0],
    #                       [0.0, 0.0, -1.0]])

    # NED -> NWU
    # x_nwu = np.array([1.0, 2.0, 3.0])
    # print(dot(inv(R_nwu_ned), x_nwu))

    # T = Transform(R=)
