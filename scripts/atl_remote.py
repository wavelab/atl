#!/usr/bin/env python
from math import pi
from time import sleep

import rospy

from atl import Camera
from atl import LandingZone
from atl import Gimbal
from atl import Quadrotor
# from atl import World
from atl import MAVROS


def lz_circle_path(radius, velocity):
    # calculate time taken to complete circle
    distance = 2 * pi * radius
    time_taken = distance / velocity

    # angular velocity needed to make circle path
    angular_velocity = (2 * pi) / time_taken

    return (velocity, angular_velocity)


def lz_circle(radius, velocity):
    velocity, angular_velocity = lz_circle_path(radius, velocity)
    lz.set_velocity(velocity)
    lz.set_angular_velocity(angular_velocity)


def lz_straight_line(velocity):
    lz.set_velocity(velocity)


def side_to_side(quad, altitude, dist):
    for i in range(10):
        side_1 = [dist, 0.0, altitude]
        quad.set_hover_point(side_1)
        print(side_1)
        sleep(5)

        side_2 = [-dist, 0.0, altitude]
        quad.set_hover_point(side_2)
        print(side_2)
        sleep(5)


def square(quad, altitude, width):
    for i in range(10):
        wp1 = [width, width, altitude]
        quad.set_hover_point(wp1)
        print(wp1)
        sleep(5)

        wp2 = [-width, width, altitude]
        quad.set_hover_point(wp2)
        print(wp2)
        sleep(5)

        wp3 = [-width, -width, altitude]
        quad.set_hover_point(wp3)
        print(wp3)
        sleep(5)

        wp4 = [width, -width, altitude]
        quad.set_hover_point(wp4)
        print(wp4)
        sleep(5)


def init_svo(lz):
    for i in range(2):
        lz.set_position([0, 0, 0])
        sleep(1)

        lz.set_position([0, -0.4, 0])
        sleep(1)

        lz.set_position([-0.4, 0, 0])
        sleep(1)

        lz.set_position([0, 0.4, 0])
        sleep(1)

        lz.set_position([0.4, 0, 0])
        sleep(1)


def hover_side_to_side():
    for i in range(10):
        quad.set_hover_point([5, 0, 5.0])
        sleep(5)
        quad.set_hover_point([-5, 0, 5.0])
        sleep(5)


def up_and_down():
    for i in range(3):
        quad.set_hover_point([0, 0, 5.0])
        sleep(5)
        quad.set_hover_point([0, 0, 4.0])
        sleep(5)


def deg2rad(d):
    return d * pi / 180.0


def rad2deg(r):
    return r * 180.0 / pi


if __name__ == "__main__":
    rospy.init_node("atl_remote")
    lz = LandingZone()
    camera = Camera()
    quad = Quadrotor()
    gimbal = Gimbal()
    mavros = MAVROS()
    # world = World()
    rospy.sleep(1.0)

    # gimbal.activate(True)
    # gimbal.set_attitude([0.0, -1.0, 0])
    gimbal.activate(False)

    # quad.set_arm(True)
    # quad.set_mode("DISCOVER_MODE")
    # quad.set_yaw(90)
    # quad.set_mode("WAYPOINT_MODE")
