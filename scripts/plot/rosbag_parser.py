#!/usr/bin/env python2
import rosbag


def parse_pose_stamped(bag, topic):
    pos_x = []
    pos_y = []
    pos_z = []

    for topic, msg, t in bag.read_messages(topics=topic):
        pos_x.append(msg.pose.position.x)
        pos_y.append(msg.pose.position.y)
        pos_z.append(msg.pose.position.z)

    return (pos_x, pos_y, pos_z)


def parse_vec3(bag, topic):
    x = []
    y = []
    z = []

    for topic, msg, t in bag.read_messages(topics=topic):
        x.append(msg.x)
        y.append(msg.y)
        z.append(msg.z)

    return (x, y, z)


def parse_quaternion(bag, topic):
    w = []
    x = []
    y = []
    z = []

    for topic, msg, t in bag.read_messages(topics=topic):
        w.append(msg.w)
        x.append(msg.x)
        y.append(msg.y)
        z.append(msg.z)

    return (w, x, y, z)
