#!/usr/bin/env python3
from math import cos
from math import sin

import random

import numpy as np
from numpy import transpose
from numpy import corrcoef
from numpy import sum
from numpy import log
from numpy import arange
from numpy.random import rand
from numpy.linalg import inv
from numpy.linalg import matrix_rank

from pylab import pcolor, show, colorbar, xticks, yticks


def frange(x, y, jump):
    retval = []

    start = x
    while start < y:
        retval.append(start)
        start += jump

    return retval


def R321(phi, theta, psi):
    return np.array([
        [
            cos(theta) * cos(psi),
            sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi),
            cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)
        ],
        [
            cos(theta) * sin(psi),
            sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi),
            cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)
        ],
        [
            -sin(theta),
            sin(phi) * cos(theta),
            cos(phi) * cos(theta)
        ]
    ])


def transform(X):
    gimbal_roll, gimbal_pitch, gimbal_yaw, tag_x, tag_y, tag_z = X

    x_c = np.array([tag_x, tag_y, tag_z])
    c_R_n = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    n_R_g = R321(gimbal_roll, gimbal_pitch, gimbal_yaw)
    g_R_p = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    return g_R_p.dot(n_R_g.dot(c_R_n.dot(x_c)))


def J(X):
    return np.array([
        [
            -sin(X[0])*cos(X[1]),
            -cos(X[1])*cos(X[0]),
            -sin(X[1]),
            -X[3]*cos(X[1])*cos(X[0]) + X[4]*sin(X[0])*cos(X[1]),
            X[3]*sin(X[1])*sin(X[0]) + X[4]*sin(X[1])*cos(X[0]) - X[5]*cos(X[1]),
            0
        ], [
            -sin(X[1])*sin(X[0])*sin(X[2]) - cos(X[0])*cos(X[2]),
            -sin(X[1])*sin(X[2])*cos(X[0]) + sin(X[0])*cos(X[2]),
            sin(X[2])*cos(X[1]),
            -X[3]*(sin(X[1])*sin(X[2])*cos(X[0]) - sin(X[0])*cos(X[2])) - X[4]*(-sin(X[1])*sin(X[0])*sin(X[2]) - cos(X[0])*cos(X[2])),
            -X[3]*sin(X[0])*sin(X[2])*cos(X[1]) - X[4]*sin(X[2])*cos(X[1])*cos(X[0]) - X[5]*sin(X[1])*sin(X[2]),
            -X[3]*(sin(X[1])*sin(X[0])*cos(X[2]) - sin(X[2])*cos(X[0])) - X[4]*(sin(X[1])*cos(X[0])*cos(X[2]) + sin(X[0])*sin(X[2])) + X[5]*cos(X[1])*cos(X[2])
        ], [
            sin(X[1])*sin(X[0])*cos(X[2]) - sin(X[2])*cos(X[0]),
            sin(X[1])*cos(X[0])*cos(X[2]) + sin(X[0])*sin(X[2]),
            -cos(X[1])*cos(X[2]),
            X[3]*(sin(X[1])*cos(X[0])*cos(X[2]) + sin(X[0])*sin(X[2])) + X[4]*(-sin(X[1])*sin(X[0])*cos(X[2]) + sin(X[2])*cos(X[0])),
            X[3]*sin(X[0])*cos(X[1])*cos(X[2]) + X[4]*cos(X[1])*cos(X[0])*cos(X[2]) + X[5]*sin(X[1])*cos(X[2]),
            X[3]*(-sin(X[1])*sin(X[0])*sin(X[2]) - cos(X[0])*cos(X[2])) + X[4]*(-sin(X[1])*sin(X[2])*cos(X[0]) + sin(X[0])*cos(X[2])) + X[5]*sin(X[2])*cos(X[1])
        ]
    ]).reshape((3, 6))


# gimbal and tag
gimbal_roll = 0.01
gimbal_pitch = 0.01
gimbal_yaw = 0.01
tag_x = 0.01
tag_y = 0.01
tag_z = 10

# x_c = np.array([tag_x, tag_y, tag_z])
# c_R_n = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
# n_R_g = R321(gimbal_roll, gimbal_pitch, gimbal_yaw)
# g_R_p = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
# f = g_R_p.dot(n_R_g.dot(c_R_n.dot(x_c)))

X = [gimbal_roll, gimbal_pitch, gimbal_yaw, tag_x, tag_y, tag_z]
J = J(X)
print(J)
print(J.transpose())

cov = inv(J.transpose().dot(J))
print(cov)


# data = [[] for i in range(6)]
# for i in range(100):
#     gimbal_roll = random.uniform(-1.0, 1.0)
#     gimbal_pitch = random.uniform(-1.0, 1.0)
#     gimbal_yaw = random.uniform(-1.0, 1.0)
#     tag_x = random.uniform(-1.0, 1.0)
#     tag_y = random.uniform(-1.0, 1.0)
#     tag_z = random.uniform(-1.0, 1.0)
#
#     X = [gimbal_roll, gimbal_pitch, gimbal_yaw, tag_x, tag_y, tag_z]
#     try:
#         cov = inv(transpose(J(X)).dot(J(X)))
#         print(cov)
#     except Exception:
#         pass

# generating some uncorrelated data
# data = rand(10, 100)  # each row of represents a variable

# creating correlation between the variables
# variable 2 is correlated with all the other variables
# data[2, :] = sum(data, 0)
# variable 4 is correlated with variable 8
# data[4, :] = log(data[8, :]) * 0.5

# plotting the correlation matrix
# R = corrcoef(cov)
# pcolor(R)
# colorbar()
# yticks(arange(0.5, 6.5), range(0, 6))
# xticks(arange(0.5, 6.5), range(0, 6))
# show()
