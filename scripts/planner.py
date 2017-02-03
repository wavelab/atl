#!/usr/bin/env python3
from math import cos
from math import sin
from math import asin

import scipy.optimize
import numpy as np
import matplotlib.pylab as plt


def quadrotor(x, u, dt):
    x[0] += x[1] * dt
    x[1] += (u[0] * sin(u[1]) - 0.5 * x[1]) * dt
    x[2] += x[3] * dt
    x[3] += (u[0] * cos(u[1]) - 10.0 - 0.2 * x[3]) * dt

    return x


def desired_system(p0, pf, T, dt):
    desired = []
    traj = []

    # calculate line equation, gradient and intersect
    m = (p0[1] - pf[1]) / (p0[0] - pf[0])
    c = p0[1] - m * p0[0]
    vx = (pf[0] - p0[0]) / (T * dt)
    vz = (pf[1] - p0[1]) / (T * dt)
    az = 10.0
    theta = asin(vx / az)

    # initial point
    traj.append(p0)
    desired.append([p0[0], vx, p0[1], vz, az, theta])

    # create points along the desired line path
    dx = (pf[0] - p0[0]) / (T - 1.0)
    for i in range(1, T - 1):
        x_prev = desired[-1]

        x = [0 for i in range(6)]
        x[0] = x_prev[0] + dx      # x
        x[1] = vx                  # vx
        x[2] = m * x[0] + c        # z
        x[3] = vz                  # vz
        x[4] = az                  # az
        x[5] = theta               # theta

        desired.append(x)
        traj.append([x[0], x[2]])

    # final point
    traj.append(pf)
    desired.append([pf[0], 0.0, pf[1], 0.0, 0.0, 0.0])

    return (np.array(desired), np.array(traj))


def plot_desired_trajectory(traj):
    traj = traj.T
    plt.plot(traj[0], traj[1])
    plt.xlabel("x")
    plt.ylabel("z")
    plt.show()


def plot_results(traj, x, T, n, m):
    # setup
    x = x.reshape(T, n + m).T
    traj = traj.T
    dt = 0.1
    t = [i * dt for i in range(T)]

    plt.figure(1)

    plt.subplot(411)
    plt.plot(traj[0], traj[1], label="desired")
    plt.plot(x[0], x[2], label="optimized")
    plt.title("Landing Trajectory")
    plt.legend(loc=0)
    plt.xlabel("x")
    plt.ylabel("z")

    plt.subplot(412)
    plt.plot(t, x[1], label="vx")
    plt.plot(t, x[3], label="vz")
    plt.title("Velocity")
    plt.legend(loc=0)
    plt.xlabel("t")
    plt.ylabel("v")

    plt.subplot(413)
    plt.title("Theta Input")
    plt.plot(t, x[5], label="theta")
    plt.legend(loc=0)
    plt.xlabel("t")
    plt.ylabel("radians")

    plt.subplot(414)
    plt.title("Thrust Input")
    plt.plot(t, x[4], label="az")
    plt.legend(loc=0)
    plt.xlabel("t")
    plt.ylabel("acceleration")

    plt.show()


def cost_func(x, args):
    cost = 0.0

    # convert the (N * T) vector to T x N matrix
    states = x.reshape(args["T"], args["n"] + args["m"]).T
    traj = args["traj"].T

    # position error cost
    cost += 1.0 * np.linalg.norm(states[0] - traj[0])  # dx
    cost += 1.0 * np.linalg.norm(states[2] - traj[1])  # dz

    # control input cost
    cost += 0.5 * np.linalg.norm(states[4])  # az
    cost += 1.0 * np.linalg.norm(states[5])  # theta

    # control input difference cost
    # cost += 0.1 * pow(np.sum(np.diff(states[4])), 2)  # az
    # cost += 0.1 * pow(np.sum(np.diff(states[5])), 2)  # theta

    return cost


def ine_constraints(x, *args):
    T = args[0]
    dt = args[1]
    n = args[2]
    m = args[3]

    # setup
    retval = np.array([])
    opt_vel = np.array([])
    fea_vel = np.array([])

    # convert the (N * T) vector to T x N matrix
    x = x.reshape(T, n + m)

    # calculate vector valued inequality constraints
    x_prev = x[0]
    for x_curr in x[1:]:
        # calculate error between optimized velocity and feasible velocity
        opt_vel = np.array(x_curr[0:4] - x_prev[0:4])
        fea_vel = np.array([
            x_prev[1],                                            # x
            x_prev[4] * sin(x_prev[5]) - 0.5 * x_prev[1],         # vx
            x_prev[3],                                            # z
            x_prev[4] * cos(x_prev[5]) - 10.0 - 0.2 * x_prev[3]   # vz
        ]) * dt

        retval = np.append(retval, fea_vel - opt_vel)
        x_prev = x_curr

    return retval


# def ine_constraints2(x, *args):
#     T = args[0]
#     dt = args[1]
#     n = args[2]
#     m = args[3]
#     i = args[4]
#
#     # setup


if __name__ == "__main__":
    T = 20      # num of time steps
    dt = 0.1    # time step
    n = 4       # num of states
    m = 2       # num of inputs

    p0 = np.array([0.0, 5.0])  # start position (x, z)
    pf = np.array([5.0, 0.0])  # end position (x, z)

    # setup
    x0, traj = desired_system(p0, pf, T, dt)
    # plot_desired_trajectory(traj)

    args = {"traj": traj, "T": T, "n": n, "m": m}
    constraints = [
        # equality constraint for start position
        {"type": "eq", "fun": lambda x: np.array([x[0] - x0[0][0]])},  # x
        {"type": "eq", "fun": lambda x: np.array([x[1] - x0[0][1]])},  # vx
        {"type": "eq", "fun": lambda x: np.array([x[2] - x0[0][2]])},  # z
        {"type": "eq", "fun": lambda x: np.array([x[3] - x0[0][3]])},  # vz
        {"type": "eq", "fun": lambda x: np.array([x[4] - x0[0][4]])},  # az
        {"type": "eq", "fun": lambda x: np.array([x[5] - x0[0][5]])},  # theta

        # equality constraint for end position
        {"type": "eq", "fun": lambda x: np.array([x[-6] - pf[0]])},  # x
        {"type": "eq", "fun": lambda x: np.array([x[-5] - 0.0])},    # vx
        {"type": "eq", "fun": lambda x: np.array([x[-4] - pf[1]])},  # z
        {"type": "eq", "fun": lambda x: np.array([x[-3] - 0.0])},    # vz
        {"type": "eq", "fun": lambda x: np.array([x[-2] - 0.0])},    # az
        {"type": "eq", "fun": lambda x: np.array([x[-1] - 0.0])},    # theta

        # nonlinear inequality constraint for motion
        {"type": "ineq", "fun": ine_constraints, "args": (T, dt, n, m)}

    ]

    result = scipy.optimize.minimize(cost_func,
                                     x0,
                                     args=args,
                                     constraints=constraints)

    # plot optimization results
    plot_results(traj, result.x, T, n, m)

    # plot real trajectory
    x = [x0[0][0], x0[0][1], x0[0][2], x0[0][3]]
    pos_x = [x[0]]
    pos_y = [x[2]]
    states = result.x.reshape(T, n + m).T
    for i in range(T):
        u = [states[4][i], states[5][i]]
        x = quadrotor(x, u, dt)
        pos_x.append(x[0])
        pos_y.append(x[2])

    plt.plot(pos_x, pos_y)
    plt.show()
