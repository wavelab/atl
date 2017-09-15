#!/usr/bin/env python2
import sys
from math import pi
from math import cos
from math import sin
from math import atan2
from math import asin

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D  # NOQA


def deg2rad(d):
    """ Convert degrees to radians """
    return d * (pi / 180.0)


def rad2deg(r):
    """ Convert radians to degrees """
    return r * (180.0 / pi)


def rotz(theta):
    """ Rotation matrix around z-axis (counter-clockwise) """
    return np.array([[cos(theta), -sin(theta), 0.0],
                     [sin(theta), cos(theta), 0.0],
                     [0.0, 0.0, 1.0]])


def euler2rot(euler, euler_seq):
    """ Convert euler to rotation matrix R
    This function assumes we are performing an extrinsic rotation.
    Source:
        Euler Angles, Quaternions and Transformation Matrices
        https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770024290.pdf
    """
    if euler_seq == 123:
        t1, t2, t3 = euler

        R11 = cos(t2) * cos(t3)
        R12 = -cos(t2) * sin(t3)
        R13 = sin(t2)

        R21 = sin(t1) * sin(t2) * cos(t3) + cos(t1) * sin(t3)
        R22 = -sin(t1) * sin(t2) * sin(t3) + cos(t1) * cos(t3)
        R23 = -sin(t1) * cos(t2)

        R31 = -cos(t1) * sin(t2) * cos(t3) + sin(t1) * sin(t3)
        R32 = cos(t1) * sin(t2) * sin(t3) + sin(t1) * cos(t3)
        R33 = cos(t1) * cos(t2)

    elif euler_seq == 321:
        t3, t2, t1 = euler

        R11 = cos(t1) * cos(t2)
        R12 = cos(t1) * sin(t2) * sin(t3) - sin(t1) * cos(t3)
        R13 = cos(t1) * sin(t2) * cos(t3) + sin(t1) * sin(t3)

        R21 = sin(t1) * cos(t2)
        R22 = sin(t1) * sin(t2) * sin(t3) + cos(t1) * cos(t3)
        R23 = sin(t1) * sin(t2) * cos(t3) - cos(t1) * sin(t3)

        R31 = -sin(t2)
        R32 = cos(t2) * sin(t3)
        R33 = cos(t2) * cos(t3)

    else:
        error_msg = "Error! Unsupported euler sequence [%s]" % str(euler_seq)
        raise RuntimeError(error_msg)

    return np.array([[R11, R12, R13],
                     [R21, R22, R23],
                     [R31, R32, R33]])


def quat2euler(q, euler_seq):
    qw, qx, qy, qz = q
    qw2 = pow(qw, 2)
    qx2 = pow(qx, 2)
    qy2 = pow(qy, 2)
    qz2 = pow(qz, 2)

    if euler_seq == 123:
        t1 = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2))
        t2 = asin(2 * (qx * qz + qy * qw))
        t3 = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2))
        return np.array([t3, t2, t1])

    elif euler_seq == 321:
        t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
        t2 = asin(2 * (qy * qw - qx * qz))
        t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

        return np.array([t1, t2, t3])

    else:
        error_msg = "Error! Unsupported euler sequence [%s]" % str(euler_seq)
        raise RuntimeError(error_msg)


def print_ros_topics(bag_path):
    bag = rosbag.Bag(bag_path)
    bag_info = bag.get_type_and_topic_info()
    import pprint
    pprint.pprint(bag_info.topics)


def plot_trajectory(scc_data):
    gps = scc_data["gps"]
    scc = scc_data["odom_pos"]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(scc[:, 0], scc[:, 1], label="SCC", color="b")
    ax.plot(gps[:, 0], gps[:, 1], label="GPS", color="r")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_xlim([-5.0, 50])
    ax.set_ylim([-10, 10])
    ax.legend()


def plot_translation_errors(scc_data):
    gps_time = scc_data["gps_time"]
    gps = scc_data["gps"]
    scc_vel_time = scc_data["vel_time"]
    scc_vel = scc_data["vel"]
    scc_pos = scc_data["odom_pos"]

    # Static stereo camera
    scc_x_total, scc_y_total, scc_z_total = calc_dist_traveled(scc_vel_time,
                                                               scc_vel)
    scc_x_error = ((gps[:, 0] - scc_pos[:, 0]) / scc_x_total) * 100.0
    scc_y_error = ((gps[:, 1] - scc_pos[:, 1]) / scc_y_total) * 100.0
    scc_z_error = ((gps[:, 2] - scc_pos[:, 2]) / scc_z_total) * 100.0

    # Plot translation errors
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(gps_time, scc_x_error, label="SCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("x - error (%)")
    ax.legend()

    ax = fig.add_subplot(312)
    ax.plot(gps_time, scc_y_error, label="SCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("y - error (%)")
    ax.legend()

    ax = fig.add_subplot(313)
    ax.plot(gps_time, scc_z_error, label="SCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("z - error (%)")
    ax.legend()


def calc_yaw_error(measured, estimated):
    yaw_error = []

    for i in range(len(measured)):
        error = measured[i] - estimated[i]
        if error > pi:
            error -= 2 * pi
        elif error < -pi:
            error += 2 * pi

        yaw_error.append(error)

    return np.array(yaw_error)


def plot_rotation_errors(scc):
    # Static stereo camera setup
    scc_roll_error = scc["imu"][:, 0] - scc["odom_att"][:, 0]
    scc_pitch_error = scc["imu"][:, 1] - scc["odom_att"][:, 1]
    scc_yaw_error = calc_yaw_error(scc["imu"][:, 2], scc["odom_att"][:, 2])
    scc_imu_time = scc["imu_time"]

    # Plot rotation errors
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(scc_imu_time, scc_roll_error, label="SCC")
    # ax.plot(mcc_imu_time, mcc_roll_error, label="MCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll - error (rad)")
    ax.legend()

    ax = fig.add_subplot(312)
    ax.plot(scc_imu_time, scc_pitch_error, label="SCC")
    # ax.plot(mcc_imu_time, mcc_pitch_error, label="MCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch - error (rad)")
    ax.legend()

    ax = fig.add_subplot(313)
    ax.plot(scc_imu_time, scc_yaw_error, label="SCC")
    # ax.plot(mcc_imu_time, mcc_yaw_error, label="MCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw - error (rad)")
    ax.legend()


def plot_gimbal_angles(gimbal_angles_t,
                       gimbal_angles,
                       ref_angles_t,
                       ref_angles):
    fig = plt.figure()
    ax = fig.add_subplot(211)
    ax.plot(gimbal_angles_t, [g[0] for g in gimbal_angles], label="Estimated roll")
    ax.plot(ref_angles_t, [g[0] for g in ref_angles], label="Measured roll")
    ax.legend()

    ax = fig.add_subplot(212)
    ax.plot(gimbal_angles_t, [g[1] for g in gimbal_angles], label="Estimated pitch")
    ax.plot(ref_angles_t, [g[1] for g in ref_angles], label="Measured pitch")
    ax.legend()


def parse_imu(bag, time_init):
    imu = []
    imu_time = []
    topics = ["/dji_sdk/attitude"]

    for topic, msg, t in bag.read_messages(topics=topics):
        quat = [msg.quaternion.w,
                msg.quaternion.x,
                msg.quaternion.y,
                msg.quaternion.z]
        imu.append(quat2euler(quat, 321))
        imu_time.append(float(t.to_sec()) - time_init)
    imu = np.array(imu)

    return (imu_time, imu)


def parse_odometry(bag, time_init):
    odom_pos = []
    odom_att = []
    odom_time = []
    topics = ["/okvis_node/okvis_odometry"]

    for topic, msg, t in bag.read_messages(topics=topics):
        position = [msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z]
        orientation = [msg.pose.pose.orientation.w,
                       msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z]

        # TRANSFORM ODOMETRY TO WORLD FRAME
        R = rotz(deg2rad(25))
        position = np.dot(R, position)

        # Convert quaternion to euler angles
        rpy = quat2euler(orientation, 321)
        rpy[2] += deg2rad(25)

        odom_pos.append(position)
        odom_att.append(rpy)
        odom_time.append(float(t.to_sec()) - time_init)

    odom_pos = np.array(odom_pos)
    odom_att = np.array(odom_att)

    return (odom_time, odom_pos, odom_att)


def parse_gps(bag, time_init):
    gps = []
    gps_time = []
    topics = ["/okvis_node/gps_odom"]

    for topic, msg, t in bag.read_messages(topics=topics):
        gps.append([msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z])
        gps_time.append(float(t.to_sec()) - time_init)
    gps = np.array(gps)

    return (gps_time, gps)


def parse_velocity(bag, time_init):
    vel = []
    vel_time = []
    topics = ["/dji_sdk/velocity"]

    for topic, msg, t in bag.read_messages(topics=topics):
        vel.append([msg.vector.x, msg.vector.y, msg.vector.z])
        vel_time.append(float(t.to_sec()) - time_init)
    vel = np.array(vel)

    return (vel_time, vel)


def parse_gimbal_angles(bag, time_init):
    gimbal_angles = []
    gimbal_angles_t = []
    ref_angles = []
    ref_angles_t = []
    topics = ["/okvis_node/gimbal_angles", "/okvis_node/ref_gimbal_angles"]

    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == "/okvis_node/gimbal_angles":
            rpy = np.array([msg.x, msg.y, msg.z])
            gimbal_angles.append(rpy)
            gimbal_angles_t.append(float(t.to_sec()) - time_init)
        elif topic == "/okvis_node/ref_gimbal_angles":
            rpy = np.array([msg.x, msg.y, msg.z])
            ref_angles.append(rpy)
            ref_angles_t.append(float(t.to_sec()) - time_init)

    gimbal_angles = np.array(gimbal_angles)
    ref_angles = np.array(ref_angles)

    return (gimbal_angles_t, gimbal_angles, ref_angles_t, ref_angles)


def interpolate_odom_gps_imu(odom_data, gps_data, imu_data):
    odom_time, odom_pos, odom_att = odom_data
    gps_time, gps = gps_data
    imu_time, imu = imu_data

    # Interpolate odometry position signal to match up against GPS signal
    f_odom_pos_x = interpolate.interp1d(odom_time, odom_pos[:, 0])
    f_odom_pos_y = interpolate.interp1d(odom_time, odom_pos[:, 1])
    f_odom_pos_z = interpolate.interp1d(odom_time, odom_pos[:, 2])
    new_odom_pos_x = f_odom_pos_x(gps_time)
    new_odom_pos_y = f_odom_pos_y(gps_time)
    new_odom_pos_z = f_odom_pos_z(gps_time)

    # Interpolate odometry attitude to match up against IMU signal
    f_odom_att_x = interpolate.interp1d(odom_time, odom_att[:, 0])
    f_odom_att_y = interpolate.interp1d(odom_time, odom_att[:, 1])
    f_odom_att_z = interpolate.interp1d(odom_time, odom_att[:, 2])
    new_odom_att_x = f_odom_att_x(imu_time)
    new_odom_att_y = f_odom_att_y(imu_time)
    new_odom_att_z = f_odom_att_z(imu_time)

    return [new_odom_pos_x, new_odom_pos_y, new_odom_pos_z,
            new_odom_att_x, new_odom_att_y, new_odom_att_z]


def calc_dist_traveled(vel_time, vel):
    t = vel_time[0]
    x = 0.0
    y = 0.0
    z = 0.0

    for i in range(1, len(vel) - 1):
        dt = vel_time[i] - t

        vx = abs(vel[i, 0])
        x += vx * dt

        vy = abs(vel[i, 1])
        y += vy * dt

        vz = abs(vel[i, 2])
        z += vz * dt

        t = vel_time[i]

    return (x, y, z)


def parse_bag(bag_path):
    bag = rosbag.Bag(bag_path)

    # Get first timestamp
    time_init = None
    for ros_topic, ros_msg, ros_time in bag.read_messages():
        time_init = float(ros_time.to_sec())
        break

    # Parse data
    gps_time, gps = parse_gps(bag, time_init)
    vel_time, vel = parse_velocity(bag, time_init)
    imu_time, imu = parse_imu(bag, time_init)
    odom_time, odom_pos, odom_att = parse_odometry(bag, time_init)
    gimbal_data = parse_gimbal_angles(bag, time_init)

    # Close ROS bag
    bag.close()

    # Interpolate odometry data
    odom_data = (odom_time, odom_pos, odom_att)
    gps_data = (gps_time, gps)
    imu_data = (imu_time, imu)
    new_odom_data = interpolate_odom_gps_imu(odom_data, gps_data, imu_data)
    interp_odom_pos = np.array(new_odom_data[0:3]).transpose()
    interp_odom_att = np.array(new_odom_data[3:6]).transpose()

    return {"gps_time": gps_time, "gps": gps, "odom_pos": interp_odom_pos,
            "imu_time": imu_time, "imu": imu, "odom_att": interp_odom_att,
            "vel_time": vel_time, "vel": vel,
            "gimbal_angles_t": gimbal_data[0], "gimbal_angles": gimbal_data[1],
            "ref_angles_t": gimbal_data[2], "ref_angles": gimbal_data[3]}


def plot_data(scc_bag_path):
    scc_data = parse_bag(scc_bag_path)

    # Set plot style
    plt.style.use("ggplot")

    # Figure 1 - trajectory plot
    plot_trajectory(scc_data)
    plt.savefig("plot_trajectory.png")

    # Figure 2 - translation errors
    plot_translation_errors(scc_data)
    plt.savefig("plot_translation_errors.png")

    # Figure 3 - rotation errors
    plot_rotation_errors(scc_data)
    plt.savefig("plot_rotation_errors.png")

    # Figure 4 - Gimbal angles measured vs estimated
    # plot_gimbal_angles(*gimbal_data)
    # plt.savefig("plot_gimbal_errors%s.png")

    plt.show()


if __name__ == "__main__":
    scc_bag_path = sys.argv[1]

    plot_data(scc_bag_path)
