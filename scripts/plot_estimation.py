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

    plt.plot(data["x"], data["y"])
    plt.plot(data["est_x"], data["est_y"])
    plt.show()


if __name__ == "__main__":
    plot(output_file)
