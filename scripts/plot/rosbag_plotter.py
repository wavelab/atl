#!/usr/bin/env python2
import matplotlib.pylab as plt


def plot_vec3_vs_time(data):
    (x, y, z) = data
    T = range(len(x))

    plt.figure(1)

    plt.subplot(411)
    plt.tight_layout()
    plt.plot(x, y)
    plt.title("x vs y")
    plt.xlabel("x")
    plt.ylabel("y")

    plt.subplot(412)
    plt.plot(T, x)
    plt.title("x vs T")
    plt.xlabel("time step")
    plt.ylabel("x")

    plt.subplot(413)
    plt.plot(T, y)
    plt.title("y vs T")
    plt.xlabel("time step")
    plt.ylabel("y")

    plt.subplot(414)
    plt.plot(T, z)
    plt.title("z vs T")
    plt.xlabel("time step")
    plt.ylabel("z")

    plt.show()


def plot_quaternion_vs_time(data):
    (w, x, y, z) = data
    T = range(len(x))

    plt.figure(1)

    plt.subplot(411)
    plt.tight_layout()
    plt.plot(T, w)
    plt.title("w vs T")
    plt.xlabel("time step")
    plt.ylabel("w")

    plt.subplot(412)
    plt.plot(T, x)
    plt.title("x vs T")
    plt.xlabel("time step")
    plt.ylabel("x")

    plt.subplot(413)
    plt.plot(T, y)
    plt.title("y vs T")
    plt.xlabel("time step")
    plt.ylabel("y")

    plt.subplot(414)
    plt.plot(T, z)
    plt.title("z vs T")
    plt.xlabel("time step")
    plt.ylabel("z")

    plt.show()
