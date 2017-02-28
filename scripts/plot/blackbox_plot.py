#!/usr/bin/env python3
import csv
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D

# GLOBAL VARIABLES
blackbox_output = "/tmp/landing_blackbox.csv"


def plot_3d(quad_data, wp_data):
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')
    ax.plot(quad_data[0], quad_data[1], quad_data[2], label="quadrotor")
    ax.plot(wp_data[0], wp_data[1], wp_data[2], label="waypoints")
    ax.legend()


def plot_blackbox(data_file):
    # setup
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header
    # next(reader)  # skip header

    data = {
        "t": [],
        "dt": [],
        "x": [], "y": [], "z": [],
        "vx": [], "vy": [], "vz": [],
        "wp_pos_x": [], "wp_pos_z": [],
        "wp_vel_x": [], "wp_vel_z": [],
        "wp_thrust": [], "wp_theta": [],
        "target_pos_bf_x": [], "target_pos_bf_y": [], "target_pos_bf_z": [],
        "target_vel_bf_x": [], "target_vel_bf_y": [], "target_vel_bf_z": [],
        "roll": [], "pitch": [], "yaw": [],
        "thrust": []
    }

    # parse data
    for row in reader:
        data["dt"].append(float(row[0]))
        data["x"].append(float(row[1]))
        data["y"].append(float(row[2]))
        data["z"].append(float(row[3]))
        data["vx"].append(float(row[4]))
        data["vy"].append(float(row[5]))
        data["vz"].append(float(row[6]))
        data["wp_pos_x"].append(float(row[7]))
        data["wp_pos_z"].append(float(row[8]))
        data["wp_vel_x"].append(float(row[9]))
        data["wp_vel_z"].append(float(row[10]))
        data["wp_thrust"].append(float(row[11]))
        data["wp_theta"].append(float(row[12]))
        data["target_pos_bf_x"].append(float(row[13]))
        data["target_pos_bf_y"].append(float(row[14]))
        data["target_pos_bf_z"].append(float(row[15]))
        data["target_vel_bf_x"].append(float(row[16]))
        data["target_vel_bf_y"].append(float(row[17]))
        data["target_vel_bf_z"].append(float(row[18]))
        data["roll"].append(float(row[19]))
        data["pitch"].append(float(row[20]))
        data["yaw"].append(float(row[21]))
        data["thrust"].append(float(row[22]))

    # convert dt to time series
    t = 0
    for i in range(len(data["dt"])):
        t += data["dt"][i]
        data["t"].append(t)

    # correct waypoints
    p0 = (data["y"][0], data["z"][0])
    for i in range(len(data["dt"])):
        data["wp_pos_x"][i] += p0[0]

    # # correct pitch
    # for i in range(len(data["dt"])):
    #     data["wp_theta"][i] = data["wp_theta"][i] = p0[0]

    # plot
    plt.style.use("ggplot")
    plt.subplot("411")
    plt.tight_layout()
    # plt.plot(data["y"], data["z"], label="quadrotor")
    # plt.plot(data["wp_pos_x"], data["wp_pos_z"], label="waypoints")
    # plt.legend(loc=0)
    # plt.title("Position vs Waypoints")
    plt.plot(data["t"], data["x"], label="x")
    plt.plot(data["t"], data["y"], label="y")
    plt.plot(data["t"], data["z"], label="z")
    plt.legend(loc=0)
    plt.title("Position vs Time")

    plt.subplot("412")
    plt.plot(data["t"], data["vy"], label="vx")
    plt.plot(data["t"], data["vz"], label="vz")
    plt.plot(data["t"], data["wp_vel_x"], label="vx_desired")
    plt.plot(data["t"], data["wp_vel_z"], label="vz_desired")
    plt.title("Waypoint Velocity vs Velocity Achieved")
    plt.legend(loc=0)

    plt.subplot("413")
    plt.plot(data["t"], data["roll"], label="roll actual")
    plt.plot(data["t"], data["pitch"], label="pitch actual")
    plt.plot(data["t"], data["yaw"], label="yaw actual")
    plt.plot(data["t"], data["wp_theta"], label="pitch desired")
    plt.title("Attitude")
    plt.legend(loc=0)

    plt.subplot("414")
    plt.plot(data["t"], data["thrust"], label="thrust actual")
    plt.plot(data["t"], data["wp_thrust"], label="thrust desired")
    plt.title("Thrust")
    plt.legend(loc=0)

    # 3d plot
    # p0 = (data["x"][0], data["y"][0], data["z"][0])
    # quad_data = (data["x"], data["y"], data["z"])
    # wp_data = [[], [], []]
    #
    # plot_3d(quad_data, wp_data)
    plt.show()


if __name__ == "__main__":
    plot_blackbox(blackbox_output)
