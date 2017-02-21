#!/usr/bin/env python2
import rosbag
import matplotlib.pylab as plt

# bag = rosbag.Bag("awesomo_bags/170220-position_controller_tuning_1.bag")
bag = rosbag.Bag("awesomo_bags/170220-position_controller_tuning_2.bag")


if __name__ == "__main__":
    pos_x = []
    pos_y = []
    pos_z = []
    topic = "/awesomo/quadrotor/pose/local"

    for topic, msg, t in bag.read_messages(topics=topic):
        pos_x.append(msg.pose.position.x)
        pos_y.append(msg.pose.position.y)
        pos_z.append(msg.pose.position.z)

    T = range(len(pos_x))

    plt.figure(1)

    plt.subplot(411)
    plt.tight_layout()
    plt.plot(pos_x, pos_y)
    plt.title("x vs y")
    plt.xlabel("x position")
    plt.ylabel("y position")

    plt.subplot(412)
    plt.plot(T, pos_x)
    plt.title("T vs x")
    plt.xlabel("time step")
    plt.ylabel("x position")

    plt.subplot(413)
    plt.plot(T, pos_y)
    plt.title("T vs y")
    plt.xlabel("time step")
    plt.ylabel("y position")

    plt.subplot(414)
    plt.plot(T, pos_z)
    plt.title("T vs z")
    plt.xlabel("time step")
    plt.ylabel("z position")

    plt.show()
