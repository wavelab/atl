#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
# output_file = "/tmp/estimation_kf_tracker_test.output"
# output_file = "/tmp/estimation_ekf_test.output"
output_file = "/tmp/estimation_ekf_tracker_test.output"


def plot(data_file):
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "timestep": [],
        "x": [],
        "y": [],
        "est_x": [],
        "est_y": []
    }

    for row in reader:
        data["timestep"].append(float(row[0]))
        data["x"].append(float(row[1]))
        data["y"].append(float(row[2]))
        data["est_x"].append(float(row[4]))
        data["est_y"].append(float(row[5]))

    plt.plot(data["x"][::100], data["y"][::100])
    plt.plot(data["est_x"][::100], data["est_y"][::100])
    plt.show()


def plot2(data_file):
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "timestep": [],
        "x": [],
        "y": [],
        "z": [],
        "theta": [],
        "v": [],
        "est_x": [],
        "est_y": [],
        "est_z": [],
        "est_theta": [],
        "est_v": []
    }

    for row in reader:
        data["timestep"].append(float(row[0]))
        data["x"].append(float(row[1]))
        data["y"].append(float(row[2]))
        data["z"].append(float(row[3]))
        data["theta"].append(float(row[4]))
        data["v"].append(float(row[5]))

        data["est_x"].append(float(row[6]))
        data["est_y"].append(float(row[7]))
        data["est_z"].append(float(row[8]))
        data["est_theta"].append(float(row[9]))
        data["est_v"].append(float(row[10]))

    plt.subplot("311")
    plt.plot(data["x"][::10], data["y"][::10], color="r", label="True")
    plt.scatter(data["est_x"][::10], data["est_y"][::10], color="b",
                label="Estimated")
    plt.title("Position")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend(loc=1)

    plt.subplot("312")
    plt.plot(data["timestep"], data["theta"], color="r", label="True")
    plt.scatter(data["timestep"][::10], data["est_theta"][::10], color="b",
                label="Estimated")
    plt.title("Theta")
    plt.xlabel("time")
    plt.ylabel("heading (radians)")
    plt.legend(loc=1)

    plt.subplot("313")
    plt.plot(data["timestep"], data["v"], color="r", label="True")
    plt.scatter(data["timestep"], data["est_v"], color="b", label="Estimated")
    plt.title("Velocity")
    plt.xlabel("time")
    plt.ylabel("velocity")
    plt.legend(loc=1)
    plt.show()

if __name__ == "__main__":
    # plot(output_file)
    plot2(output_file)
