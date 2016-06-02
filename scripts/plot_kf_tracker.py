#!/usr/bin/env python2
import pandas
import matplotlib.pylab as plt


if __name__ == "__main__":
    df = pandas.read_csv("estimator_test.dat")

    x = df["x"]
    y = df["y"]
    bx = df["bx"]
    by = df["by"]

    df.plot(x="time_step")
    plt.show()
