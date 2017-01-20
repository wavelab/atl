#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
kf_tracker_output = "/tmp/estimation_kf_tracker_test.output"
ekf_output = "/tmp/estimation_ekf_test.output"



def parse_data(data_file):
    f = open(data_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "timestep": [],
        "x": [],
        "y": []
    }

    for row in reader:
        data["timestep"].append(float(row[0]))
        data["x"].append(float(row[1]))
        data["y"].append(float(row[2]))

    plt.plot(data["x"], data["y"])
    plt.show()


# def plot(data_file):
    # df = pandas.read_csv(data_file)

    # x = df["x"]
    # y = df["y"]
    # bx = df["bx"]
    # by = df["by"]

    # df.plot(x="time_step")
    # plt.show()

if __name__ == "__main__":
    parse_data(ekf_output)
    # plot(kf_tracker_output)
    # plot(ekf_output)
