#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
trajectory_output = "/tmp/trajectory.output"
path_output = "/tmp/path.output"
optimized_output = "/tmp/trajectory_optimized.output"


def plot_trajectory(data_file):
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "timestep": [],
        "x": [],
        "z": []
    }

    for row in reader:
        data["timestep"].append(float(row[0]))
        data["x"].append(float(row[1]))
        data["z"].append(float(row[2]))

    plt.plot(data["x"], data["z"])
    plt.show()


def plot_path(data_file):
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "x": [],
        "z": []
    }

    for row in reader:
        data["x"].append(float(row[0]))
        data["z"].append(float(row[2]))

    plt.plot(data["x"], data["z"])
    plt.show()


def plot_optimized(data_file):
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "x": [],
        "z": []
    }

    for row in reader:
        data["x"].append(float(row[1]))
        data["z"].append(float(row[3]))

    plt.plot(data["x"], data["z"])
    plt.show()


if __name__ == "__main__":
    # plot_trajectory(trajectory_output)
    # plot_path(path_output)
    plot_optimized(optimized_output)
