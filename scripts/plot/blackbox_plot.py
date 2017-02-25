#!/usr/bin/env python3
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
blackbox_output = "/tmp/landing_blackbox.csv"


def plot_blackbox(data_file):
    # setup
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "t": [],
        "dt": [],
        "x": [], "y": [], "z": [],
        "vx": [], "vy": [], "vz": [],
        "wp_pos_x": [], "wp_pos_y": [],
        "wp_vel_x": [], "wp_vel_y": [],
        "target_pos_bf_x": [], "target_pos_bf_y": [], "target_pos_bf_z": [],
        "target_vel_bf_x": [], "target_vel_bf_y": [], "target_vel_bf_z": []
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
        data["wp_pos_y"].append(float(row[8]))
        data["wp_vel_x"].append(float(row[9]))
        data["wp_vel_y"].append(float(row[10]))
        data["target_pos_bf_x"].append(float(row[11]))
        data["target_pos_bf_y"].append(float(row[12]))
        data["target_pos_bf_z"].append(float(row[13]))
        data["target_vel_bf_x"].append(float(row[14]))
        data["target_vel_bf_y"].append(float(row[15]))
        data["target_vel_bf_z"].append(float(row[16]))

    # convert dt to time series
    t = 0
    for i in range(len(data["dt"])):
        t += data["dt"][i]
        data["t"].append(t)

    # correct waypoints
    p0 = (data["y"][0], data["z"][0])
    for i in range(len(data["dt"])):
        data["wp_pos_x"][i] += p0[0]

    # plot
    plt.subplot("311")
    plt.tight_layout()
    plt.plot(data["y"], data["z"], label="quadrotor")
    plt.plot(data["wp_pos_x"], data["wp_pos_y"], label="waypoints")
    plt.legend(loc=0)
    plt.title("Position vs Waypoints")

    plt.subplot("312")
    plt.plot(data["t"], data["vy"], label="vy")
    plt.plot(data["t"], data["vz"], label="vz")
    plt.plot(data["t"], data["wp_vel_x"], label="vx_desired")
    plt.plot(data["t"], data["wp_vel_y"], label="vz_desired")
    plt.title("Waypoint Velocity vs Velocity Achieved")
    plt.legend(loc=0)

    plt.subplot("313")
    plt.plot(data["t"], data["vy"], label="vy")
    plt.plot(data["t"], data["vz"], label="vz")
    plt.plot(data["t"], data["target_vel_bf_x"], label="vx_desired")
    plt.plot(data["t"], data["target_vel_bf_z"], label="vz_desired")
    plt.title("Target Velocity vs Velocity Achieved")
    plt.legend(loc=0)

    plt.show()


if __name__ == "__main__":
    plot_blackbox(blackbox_output)
