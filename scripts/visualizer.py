#!/usr/bin/env python2
import csv
from math import cos
from math import sin
from math import radians

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class QuadrotorVisualizer(object):
    def __init__(self, **kwargs):
        # data
        self.data_path = kwargs["data_path"]
        self.data = None
        self.load_data()

        # plot variables
        self.fig = None
        self.ax = None

        # quadrotor properties
        self.position = [0, 0, 0]
        self.arm_length = kwargs.get("arm_length", 1)
        self.arm_1 = [0, 0, 0]
        self.arm_2 = [0, 0, 0]
        self.arm_3 = [0, 0, 0]
        self.arm_4 = [0, 0, 0]
        self.arms = [self.arm_1, self.arm_2, self.arm_3, self.arm_4]
        self.collection = []

    def load_data(self):
        # setup
        self.data = {
            "x": [],
            "y": [],
            "z": [],
            "roll": [],
            "pitch": [],
            "yaw": []
        }
        csv_file = open(self.data_path, "r")
        csv_reader = csv.reader(csv_file, delimiter=',')

        # load data
        self.data["length"] = 0
        for line in csv_reader:
            self.data["x"].append(float(line[0]))
            self.data["y"].append(float(line[1]))
            self.data["z"].append(float(line[2]))
            self.data["roll"].append(float(line[3]))
            self.data["pitch"].append(float(line[4]))
            self.data["yaw"].append(float(line[5]))
            self.data["length"] += 1

        # clean up
        csv_file.close()

    def setup_plot(self):
        # setup
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

        # plot settings
        plt.axis("equal")

        # plot labels
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        # plot limits
        self.ax.set_xlim((-20, 20))
        self.ax.set_ylim((-20, 20))
        self.ax.set_zlim((0, 20))

    def plot_arms(self):
        # pre-check
        if self.ax is None:
            raise RuntimeError("ax cannot be None! did you setup plot?")

        # clear collection
        if len(self.collection) != 0:
            for line in self.collection:
                line.remove()
            self.collection = []

        # plot each arm
        for arm in self.arms:
            self.collection.append(
                self.ax.plot(
                    [self.position[0], arm[0]],
                    [self.position[1], arm[1]],
                    [self.position[2], arm[2]],
                    c="r",
                    marker="o"
                )[0]
            )

    def rotate(self, vec, roll, pitch, yaw):
        # assumes angles are in radians
        a = yaw
        b = pitch
        g = roll

        # rotation matrix
        rot_mat = np.matrix([
            # row 1
            [
                cos(a) * cos(b),
                cos(a) * sin(b) * sin(g) - sin(a) * cos(g),
                cos(a) * sin(b) * cos(g) + sin(a) * sin(g)
            ],

            # row 2
            [
                sin(a) * cos(b),
                sin(a) * sin(b) * sin(g) + cos(a) * cos(g),
                sin(a) * sin(b) * cos(g) - cos(a) * sin(g)
            ],

            # row 3
            [
                -1 * sin(b),
                cos(b) * sin(g),
                cos(b) * cos(g)
            ]
        ])

        result = np.dot(rot_mat, vec)
        return np.squeeze(np.asarray(result))

    def update(self, i):
        # update position
        self.position[0] = self.data["x"][i]
        self.position[1] = self.data["y"][i]
        self.position[2] = self.data["z"][i]

        # set arm positions relative to the body
        self.arm_1[0] = self.arm_length
        self.arm_1[1] = 0.0
        self.arm_1[2] = 0.0

        self.arm_2[0] = 0.0
        self.arm_2[1] = -1.0 * self.arm_length
        self.arm_2[2] = 0.0

        self.arm_3[0] = -1.0 * self.arm_length
        self.arm_3[1] = 0.0
        self.arm_3[2] = 0.0

        self.arm_4[0] = 0.0
        self.arm_4[1] = 1.0 * self.arm_length
        self.arm_4[2] = 0.0

        # rotate according to roll, pitch and yaw
        roll = self.data["roll"][i]
        pitch = self.data["pitch"][i]
        yaw = self.data["yaw"][i]
        self.arm_1 = self.rotate(self.arm_1, roll, pitch, yaw)
        self.arm_2 = self.rotate(self.arm_2, roll, pitch, yaw)
        self.arm_3 = self.rotate(self.arm_3, roll, pitch, yaw)
        self.arm_4 = self.rotate(self.arm_4, roll, pitch, yaw)
        self.arms = [self.arm_1, self.arm_2, self.arm_3, self.arm_4]

        # translate arms relative to the world
        self.arm_1[0] += self.position[0]
        self.arm_1[1] += self.position[1]
        self.arm_1[2] += self.position[2]

        self.arm_2[0] += self.position[0]
        self.arm_2[1] += self.position[1]
        self.arm_2[2] += self.position[2]

        self.arm_3[0] += self.position[0]
        self.arm_3[1] += self.position[1]
        self.arm_3[2] += self.position[2]

        self.arm_4[0] += self.position[0]
        self.arm_4[1] += self.position[1]
        self.arm_4[2] += self.position[2]

        # plot
        if self.ax:
            self.plot_arms()
            plt.draw()

    def visualize(self):
        if self.ax is None:
            self.setup_plot()

        x = animation.FuncAnimation(
            self.fig,
            self.update,
            range(self.data["length"]),
            interval=0
        )
        x  # its weird without setting x, matplotlib wont plot
        plt.gca().set_aspect("equal", adjustable="box")
        plt.show()


if __name__ == "__main__":
    q = QuadrotorVisualizer(data_path="/tmp/sim.out")
    q.visualize()
