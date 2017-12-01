#!/bin/python3
import csv

import matplotlib.pylab as plt

BLACKBOX_FILE = "/tmp/blackbox.dat"


def plot(blackbox_file):
    # open waypoints file
    blackbox_output = open(blackbox_file, "r")
    csv_reader = csv.reader(blackbox_output, delimiter=",")
    next(csv_reader)

    # parse waypoints data
    data = {"x": [], "y": [], "z": [],
            "wp_x": [], "wp_y": [], "wp_z": []}
    for row in csv_reader:
        data["x"].append(float(row[0]))
        data["y"].append(float(row[1]))
        data["z"].append(float(row[2]))
        data["wp_x"].append(float(row[3]))
        data["wp_y"].append(float(row[4]))
        data["wp_z"].append(float(row[5]))

    # plot waypoints
    plt.scatter(data["x"], data["y"], label="state")
    plt.scatter(data["wp_x"], data["wp_y"], label="waypoints")


if __name__ == "__main__":
    plot(BLACKBOX_FILE)

    plt.legend(loc=0)
    plt.xlabel("x - position (m)")
    plt.ylabel("y - position (m)")
    plt.show()
