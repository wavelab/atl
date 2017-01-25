#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
trajectory_output = "/tmp/trajectory.output"


def plot_data(data_file):
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


if __name__ == "__main__":
    plot_data(trajectory_output)
