#!/usr/bin/env python2
import pandas
import matplotlib.pylab as plt

# GLOBAL VARIABLES
kf_tracker_output = "/tmp/estimation_kf_tracker_test.output"


def plot(data_file):
    df = pandas.read_csv(data_file)

    # x = df["x"]
    # y = df["y"]
    # bx = df["bx"]
    # by = df["by"]

    df.plot(x="time_step")
    plt.show()

if __name__ == "__main__":
    plot(kf_tracker_output)
