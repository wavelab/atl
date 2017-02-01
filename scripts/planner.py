#!/usr/bin/env python3
from math import cos
from math import sin
# from matplotlib.pylab import plot


def quadrotor(x, u, dt):
    x[0] = x[0] + u[0] * cos(x[2]) * dt
    x[1] = x[1] + u[0] * sin(x[2]) * dt
    x[2] = x[2] + u[1] * dt

    return x


if __name__ == "__main__":
    x = [0.0, 0.0, 0.0]
    u = [10.0, 0.0]
    dt = 0.1

    x = quadrotor(x, u, dt)
