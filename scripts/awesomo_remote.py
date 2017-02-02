#!/usr/bin/env python
from math import pi

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String
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


class Camera(ROSNode):
    def __init__(self):
        super(Camera, self).__init__()
        self.image_topic = "/awesomo/camera/image"
        self.mode_topic = "/prototype/camera/mode"

        self.register_publisher(self.mode_topic, String)

    def set_mode(self, mode):
        msg = String()
        msg.data = mode
        self.pubs[self.mode_topic].publish(msg)


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


class Gimbal(ROSNode):
    def __init__(self):
        super(Gimbal, self).__init__()
        self.attitude_topic = "/awesomo/gimbal/setpoint/attitude"
        self.track_topic = "/awesomo/gimbal/track"

        self.register_publisher(self.attitude_topic, Vector3)
        self.register_publisher(self.track_topic, Vector3)

    def set_attitude(self, attitude):
        msg = Vector3()
        msg.x = attitude[0]
        msg.y = attitude[1]
        msg.z = 0.0
        self.pubs[self.attitude_topic].publish(msg)

    def track(self, track):
        msg = Vector3()
        msg.x = track[0]
        msg.y = track[1]
        msg.z = 0.0
        self.pubs[self.track_topic].publish(msg)


class Quadrotor(ROSNode):
    def __init__(self):
        super(Quadrotor, self).__init__()
        self.mode_topic = "/awesomo/control/mode"
        self.heading_topic = "/awesomo/control/heading/set"
        self.hover_point_topic = "/awesomo/control/hover/set"
        self.hover_height_topic = "/awesomo/control/hover/height/set"

        self.register_publisher(self.mode_topic, String)
        self.register_publisher(self.heading_topic, Float64)
        self.register_publisher(self.hover_point_topic, Vector3)
        self.register_publisher(self.hover_height_topic, Float64)

    def set_mode(self, mode):
        msg = String()
        msg.data = mode
        self.pubs[self.mode_topic].publish(msg)

    def set_heading(self, heading):
        msg = Float64()
        msg.data = heading * (pi / 180.0)
        self.pubs[self.heading_topic].publish(msg)

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
    camera = Camera()
    quad = Quadrotor()
    gimbal = Gimbal()
    rospy.sleep(1)

    # camera.set_mode("320x320")
    # camera.set_mode("160x160")

    gimbal.set_attitude([0.0, -1.57])

    # lz.set_velocity(0.5)
    # lz.set_position([1000, 0, 0])

    velocity, angular_velocity = lz_circle_path(5, 1.0)
    lz.set_velocity(velocity)
    lz.set_angular_velocity(angular_velocity)

    # quad.set_mode("HOVER_MODE")
    # quad.set_heading(270.0)
    # quad.set_mode("DISCOVER_MODE")
    # quad.set_hover_point([3.0, -3.0, 2.0])
    # quad.set_hover_point([0.0, 0.0, 5.0])
    # quad.set_hover_height(10.0)
