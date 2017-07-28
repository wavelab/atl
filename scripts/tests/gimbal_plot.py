#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

GIMBAL_ATTITUDE_CONTROLLER_OUTPUT = "/tmp/gimbal_attitude_controller.dat"


def plot_gimbal_attitude_data():
    attitude_data = {
        "t": [],
        "roll": [],
        "pitch": []
    }

    attitude_output = open(GIMBAL_ATTITUDE_CONTROLLER_OUTPUT, "r")
    attitude_reader = csv.reader(attitude_output, delimiter=",")
    next(attitude_reader)

    for row in attitude_reader:
        attitude_data["t"].append(float(row[0]))
        attitude_data["roll"].append(float(row[1]))
        attitude_data["pitch"].append(float(row[2]))

    plt.plot(attitude_data["t"], attitude_data["roll"], label="roll")
    plt.plot(attitude_data["t"], attitude_data["pitch"], label="pitch")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Angle (degrees)")
    plt.legend(loc=0)
    plt.show()


if __name__ == "__main__":
    plot_gimbal_attitude_data()
