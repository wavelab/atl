#!/bin/python3
import csv

import matplotlib.pylab as plt

WAYPOINTS_FILE = "/tmp/waypoints.dat"
STATE_FILE = "/tmp/state.dat"


def plot_waypoints(waypoints_file):
    # open waypoints file
    waypoints_output = open(waypoints_file, "r")
    waypoints_reader = csv.reader(waypoints_output, delimiter=",")

    # parse waypoints data
    waypoints_data = {"x": [], "y": [], "z": []}
    for row in waypoints_reader:
        waypoints_data["x"].append(float(row[0]))
        waypoints_data["y"].append(float(row[1]))
        waypoints_data["z"].append(float(row[2]))

    # plot waypoints
    plt.scatter(waypoints_data["x"], waypoints_data["y"], label="waypoints")


def plot_state(state_file):
    # open state file
    state_output = open(state_file, "r")
    state_reader = csv.reader(state_output, delimiter=",")

    # parse state data
    state_data = {"x": [], "y": [], "z": [],
                  "wp_x": [], "wp_y": [], "wp_z": []}
    for row in state_reader:
        state_data["x"].append(float(row[0]))
        state_data["y"].append(float(row[1]))
        state_data["z"].append(float(row[2]))
        state_data["wp_x"].append(float(row[3]))
        state_data["wp_y"].append(float(row[4]))
        state_data["wp_z"].append(float(row[5]))

    # plot waypoints
    plt.scatter(state_data["x"], state_data["y"], label="state")
    plt.scatter(state_data["wp_x"], state_data["wp_y"], label="carrot")


if __name__ == "__main__":
    plot_waypoints(WAYPOINTS_FILE)
    plot_state(STATE_FILE)

    plt.legend(loc=0)
    plt.xlabel("x - position (m)")
    plt.ylabel("y - position (m)")
    plt.show()
