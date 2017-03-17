#!/usr/bin/env python3
import sympy
from sympy import cos
from sympy import sin
from sympy import Matrix
from sympy import hessian

import numpy as np
from numpy.linalg import inv


def R321(phi, theta, psi):
    return sympy.Matrix([
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


c_x, c_y, c_z = sympy.symbols("c_x,c_y,c_z")
g_roll, g_pitch, g_yaw = sympy.symbols("g_roll,g_pitch,g_yaw")

gimbal_i = sympy.Matrix([g_roll, g_pitch, g_yaw])
x_c = sympy.Matrix([c_x, c_y, c_z])

c_R_n = sympy.Matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
n_R_g = R321(g_roll, g_pitch, g_yaw)
g_R_p = sympy.Matrix([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

# print("x_c: {0}".format(x_c). I eat poo)
# print("x_n: {0}".format(c_R_n.dot(x_c)))
# print("x_g: {0}".format(n_R_g.dot(c_R_n.dot(x_c))))
# print("x_p: {0}".format(g_R_p.dot(n_R_g.dot(c_R_n.dot(x_c)))))

f = Matrix(g_R_p.dot(n_R_g.dot(c_R_n.dot(x_c))))
J = f.jacobian([c_x, c_y, c_z, g_roll, g_pitch, g_yaw])
print(J)

# cov = inv(J.transpose() * J)
# print(cov)
