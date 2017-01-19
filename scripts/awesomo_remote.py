#!/usr/bin/env python
from math import pi

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


class ROSNode(object):
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def register_publisher(self, topic, msg_type, queue_size=1):
        self.pubs[topic] = rospy.Publisher(topic,
                                           msg_type,
                                           queue_size=queue_size)

    def register_subscriber(self, topic, msg_type, callback):
        self.subs[topic] = rospy.Subscriber(topic, msg_type, callback)


class LandingZone(ROSNode):
    def __init__(self):
        super(LandingZone, self).__init__()
        self.position_topic = "/awesomo/lz/position/set"
        self.velocity_topic = "/awesomo/lz/velocity/set"
        self.angular_velocity_topic = "/awesomo/lz/angular_velocity/set"

        self.register_publisher(self.position_topic, Vector3)
        self.register_publisher(self.velocity_topic, Float64)
        self.register_publisher(self.angular_velocity_topic, Float64)

    def set_position(self, pos):
        msg = Vector3()
        msg.x = pos[0]
        msg.y = pos[1]
        msg.z = pos[2]
        self.pubs[self.position_topic].publish(msg)

    def set_velocity(self, vel):
        msg = Float64()
        msg.data = vel
        self.pubs[self.velocity_topic].publish(msg)

    def set_angular_velocity(self, ang_vel):
        msg = Float64()
        msg.data = ang_vel
        self.pubs[self.angular_velocity_topic].publish(msg)


class Quadrotor(ROSNode):
    def __init__(self):
        super(Quadrotor, self).__init__()
        self.hover_point_topic = "/awesomo/control/hover/set"
        self.hover_height_topic = "/awesomo/control/hover/height/set"

        self.register_publisher(self.hover_point_topic, Vector3)
        self.register_publisher(self.hover_height_topic, Float64)

    def set_hover_point(self, hover_point):
        msg = Vector3()
        msg.x = hover_point[0]
        msg.y = hover_point[1]
        msg.z = hover_point[2]
        self.pubs[self.hover_point_topic].publish(msg)

    def set_hover_height(self, hover_height):
        msg = Float64()
        msg.data = hover_height
        self.pubs[self.hover_height_topic].publish(msg)


def lz_circle_path(radius, velocity):
    # calculate time taken to complete circle
    distance = 2 * pi * radius
    time_taken = distance / velocity

    # angular velocity needed to make circle path
    angular_velocity = (2 * pi) / time_taken

    return (velocity, angular_velocity)


if __name__ == "__main__":
    rospy.init_node("awesomo_remote")
    lz = LandingZone()
    quad = Quadrotor()
    rospy.sleep(1)

    velocity, angular_velocity = lz_circle_path(30, 5.0)
    # lz.set_position([0, 0, 0])
    lz.set_velocity(velocity)
    lz.set_angular_velocity(angular_velocity)

    # quad.set_hover_point([0.0, 0.0, 10.0])
    # quad.set_hover_height(15.0)
