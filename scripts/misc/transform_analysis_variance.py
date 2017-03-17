#!/usr/bin/env python3
from math import cos
from math import sin

import numpy as np

from SALib.sample import saltelli
from SALib.analyze import sobol

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


problem = {
    'num_vars': 6,
    'names': [
        'gimbal_roll',
        'gimbal_pitch',
        'gimbal_yaw',
        'tag_x',
        'tag_y',
        'tag_z'
    ],
    'bounds': [
        [-3.14159265359, 3.14159265359],
        [-3.14159265359, 3.14159265359],
        [-3.14159265359, 3.14159265359],
        [-1.0, 1.0],
        [-1.0, 1.0],
        [-1.0, 1.0]
    ]
}
param_values = saltelli.sample(problem, 1000, calc_second_order=True)

Y = np.empty([param_values.shape[0]])
for i, X in enumerate(param_values):
    Y[i] = transform(X)[0]

Si = sobol.analyze(problem, Y, print_to_console=False)

print(Si["S2"][0, 1])
print(Si["S2"][0, 2])
print(Si["S2"][1, 2])
print(Si["S2"][3, 4])
print(Si["S2"][3, 5])
print(Si["S2"][4, 5])

# gimbal_rpy = [0.0, 0.0, 0.0]
# tag_xyz = [0, 0, 10]
# print(transform(gimbal_rpy, tag_xyz))
