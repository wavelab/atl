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
from math import sqrt
from scipy import interpolate
from sklearn.metrics import mean_squared_error
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


def plot_trajectory(mono_data, scc_data, dcc_data):
    mono_gps = mono_data["gps"]
    mono = mono_data["odom_pos"]
    scc_gps = scc_data["gps"]
    scc = scc_data["odom_pos"]
    dcc_gps = dcc_data["gps"]
    dcc = dcc_data["odom_pos"]

    fig = plt.figure()
    plt.plot(mono_gps[:, 0], mono_gps[:, 1], label="GPS", color="r")
    plt.plot(mono[:, 0], mono[:, 1], label="Mono", color="b")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.xlim([-5.0, 40])
    plt.ylim([-11, 15])
    plt.axis('equal')
    plt.legend()

    fig = plt.figure()
    plt.plot(scc_gps[:, 0], scc_gps[:, 1], label="GPS", color="r")
    plt.plot(scc[:, 0], scc[:, 1], label="SCC", color="b")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.xlim([-5.0, 40])
    plt.ylim([-11, 15])
    plt.axis('equal')
    plt.legend()

    fig = plt.figure()
    plt.plot(dcc_gps[:, 0], dcc_gps[:, 1], label="GPS", color="r")
    plt.plot(dcc[:, 0], dcc[:, 1], label="DCC", color="b")
    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.xlim([-5.0, 40])
    plt.ylim([-11, 15])
    plt.axis('equal')
    plt.legend()


def plot_translation_errors(mono, scc, dcc):
    # Mono
    mono_gps_time = mono["gps_time"]
    mono_gps = mono["gps"]
    mono_vel_time = mono["vel_time"]
    mono_vel = mono["vel"]
    mono_pos = mono["odom_pos"]
    mono_x_total, mono_y_total, mono_z_total = calc_dist_traveled(mono_vel_time,
                                                                  mono_vel)
    mono_x_error = ((mono_gps[:, 0] - mono_pos[:, 0]) / mono_x_total) * 100.0
    mono_y_error = ((mono_gps[:, 1] - mono_pos[:, 1]) / mono_y_total) * 100.0
    mono_z_error = ((mono_gps[:, 2] - mono_pos[:, 2]) / mono_z_total) * 100.0

    # mono_error = []
    # for i in range(mono_gps.shape[0]):
    #     error = mono_gps[i, :] - mono_pos[i, :]
    #     np.linalg.norm(error)
    #     mono_error.append(error)

    # mono_x_rmse_normalized = sqrt(mean_squared_error(mono_gps[:, 0], mono_pos[:, 0])) / mono_x_total
    # mono_y_rmse_normalized = sqrt(mean_squared_error(mono_gps[:, 1], mono_pos[:, 1])) / mono_y_total
    # mono_z_rmse_normalized = sqrt(mean_squared_error(mono_gps[:, 2], mono_pos[:, 2])) / mono_z_total
    # print("Mono normalized error: ", (mono_x_rmse_normalized,
    #                                   mono_y_rmse_normalized,
    #                                   mono_z_rmse_normalized))

    # Static camera cluster
    scc_gps_time = scc["gps_time"]
    scc_gps = scc["gps"]
    scc_vel_time = scc["vel_time"]
    scc_vel = scc["vel"]
    scc_pos = scc["odom_pos"]
    scc_x_total, scc_y_total, scc_z_total = calc_dist_traveled(scc_vel_time,
                                                               scc_vel)
    scc_x_error = ((scc_gps[:, 0] - scc_pos[:, 0]) / scc_x_total) * 100.0
    scc_y_error = ((scc_gps[:, 1] - scc_pos[:, 1]) / scc_y_total) * 100.0
    scc_z_error = ((scc_gps[:, 2] - scc_pos[:, 2]) / scc_z_total) * 100.0

    # scc_x_rmse_normalized = sqrt(mean_squared_error(scc_gps[:, 0], scc_pos[:, 0])) / scc_x_total
    # scc_y_rmse_normalized = sqrt(mean_squared_error(scc_gps[:, 1], scc_pos[:, 1])) / scc_y_total
    # scc_z_rmse_normalized = sqrt(mean_squared_error(scc_gps[:, 2], scc_pos[:, 2])) / scc_z_total
    # print("SCC normalized error: ", (scc_x_rmse_normalized,
    #                                  scc_y_rmse_normalized,
    #                                  scc_z_rmse_normalized))

    # Dynamic camera cluster
    dcc_gps_time = dcc["gps_time"]
    dcc_gps = dcc["gps"]
    dcc_vel_time = dcc["vel_time"]
    dcc_vel = dcc["vel"]
    dcc_pos = dcc["odom_pos"]
    dcc_x_total, dcc_y_total, dcc_z_total = calc_dist_traveled(dcc_vel_time,
                                                               dcc_vel)
    dcc_x_error = ((dcc_gps[:, 0] - dcc_pos[:, 0]) / dcc_x_total) * 100.0
    dcc_y_error = ((dcc_gps[:, 1] - dcc_pos[:, 1]) / dcc_y_total) * 100.0
    dcc_z_error = ((dcc_gps[:, 2] - dcc_pos[:, 2]) / dcc_z_total) * 100.0

    # print (dcc_x_total, dcc_y_total, dcc_z_total)
    #
    # dcc_x_rmse_normalized = sqrt(mean_squared_error(dcc_gps[:, 0], dcc_pos[:, 0])) / dcc_x_total
    # dcc_y_rmse_normalized = sqrt(mean_squared_error(dcc_gps[:, 1], dcc_pos[:, 1])) / dcc_y_total
    # dcc_z_rmse_normalized = sqrt(mean_squared_error(dcc_gps[:, 2], dcc_pos[:, 2])) / dcc_z_total
    # print("DCC normalized error: ", (dcc_x_rmse_normalized,
    #                                  dcc_y_rmse_normalized,
    #                                  dcc_z_rmse_normalized))

    # Plot translation errors
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(mono_gps_time, mono_x_error, label="Mono")
    ax.plot(scc_gps_time, scc_x_error, label="SCC")
    ax.plot(dcc_gps_time, dcc_x_error, label="DCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("x - error (%)")
    ax.legend()

    ax = fig.add_subplot(312)
    ax.plot(mono_gps_time, mono_y_error, label="Mono")
    ax.plot(scc_gps_time, scc_y_error, label="SCC")
    ax.plot(dcc_gps_time, dcc_y_error, label="DCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("y - error (%)")
    ax.legend()

    ax = fig.add_subplot(313)
    ax.plot(mono_gps_time, mono_z_error, label="Mono")
    ax.plot(scc_gps_time, scc_z_error, label="SCC")
    ax.plot(dcc_gps_time, dcc_z_error, label="DCC")
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


def plot_rotation_errors(mono, scc):
    # Mono
    mono_roll_error = mono["imu"][:, 0] - mono["odom_att"][:, 0]
    mono_pitch_error = mono["imu"][:, 1] - mono["odom_att"][:, 1]
    mono_yaw_error = calc_yaw_error(mono["imu"][:, 2], mono["odom_att"][:, 2])
    mono_imu_time = mono["imu_time"]

    # Static camera cluster
    scc_roll_error = scc["imu"][:, 0] - scc["odom_att"][:, 0]
    scc_pitch_error = scc["imu"][:, 1] - scc["odom_att"][:, 1]
    scc_yaw_error = calc_yaw_error(scc["imu"][:, 2], scc["odom_att"][:, 2])
    scc_imu_time = scc["imu_time"]

    # Dynamic camera cluster
    # dcc_roll_error = dcc["imu"][:, 0] - dcc["odom_att"][:, 0]
    # dcc_pitch_error = dcc["imu"][:, 1] - dcc["odom_att"][:, 1]
    # dcc_yaw_error = calc_yaw_error(dcc["imu"][:, 2], dcc["odom_att"][:, 2])
    # dcc_imu_time = dcc["imu_time"]

    # Plot rotation errors
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.plot(mono_imu_time, mono_roll_error, label="Mono")
    ax.plot(scc_imu_time, scc_roll_error, label="SCC")
    # ax.plot(dcc_imu_time, dcc_roll_error, label="DCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Roll - error (rad)")
    ax.legend()

    ax = fig.add_subplot(312)
    ax.plot(mono_imu_time, mono_pitch_error, label="Mono")
    ax.plot(scc_imu_time, scc_pitch_error, label="SCC")
    # ax.plot(dcc_imu_time, dcc_pitch_error, label="DCC")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch - error (rad)")
    ax.legend()

    ax = fig.add_subplot(313)
    ax.plot(mono_imu_time, mono_yaw_error, label="Mono")
    ax.plot(scc_imu_time, scc_yaw_error, label="SCC")
    # ax.plot(dcc_imu_time, dcc_yaw_error, label="DCC")
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


def parse_odometry(bag, time_init, yaw_offset):
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
        R = rotz(deg2rad(yaw_offset))
        position = np.dot(R, position)

        # Convert quaternion to euler angles
        rpy = quat2euler(orientation, 321)
        rpy[2] += deg2rad(yaw_offset)

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
            rpy = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
            gimbal_angles.append(rpy)
            gimbal_angles_t.append(float(t.to_sec()) - time_init)
        elif topic == "/okvis_node/ref_gimbal_angles":
            rpy = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
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


def parse_bag(bag_path, **kwargs):
    debug = kwargs.get("debug", False)
    trim_last = kwargs.get("trim_last", -1)
    yaw_offset = kwargs.get("yaw_offset", 0)
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
    odom_time, odom_pos, odom_att = parse_odometry(bag,
                                                   time_init,
                                                   yaw_offset)
    gimbal_data = parse_gimbal_angles(bag, time_init)

    # Plot data
    if debug:
        # Plot odometry position
        plt.figure()
        plt.subplot(311)
        plt.plot(odom_time, odom_pos[:, 0], label="OKVIS")
        plt.subplot(312)
        plt.plot(odom_time, odom_pos[:, 1], label="OKVIS")
        plt.subplot(313)
        plt.plot(odom_time, odom_pos[:, 2], label="OKVIS")

        # Plot odometry attitude
        plt.figure()
        plt.subplot(311)
        plt.plot(odom_time, odom_att[:, 0], label="OKVIS")
        plt.subplot(312)
        plt.plot(odom_time, odom_att[:, 1], label="OKVIS")
        plt.subplot(313)
        plt.plot(odom_time, odom_att[:, 2], label="OKVIS")

        # Plot imu attitude
        plt.figure()
        plt.subplot(311)
        plt.plot(imu_time, imu[:, 0], label="OKVIS")
        plt.subplot(312)
        plt.plot(imu_time, imu[:, 1], label="OKVIS")
        plt.subplot(313)
        plt.plot(imu_time, imu[:, 2], label="OKVIS")
        plt.show()

    # Close ROS bag
    bag.close()

    # Trim GPS and IMU
    gps_time = gps_time[0:trim_last]
    gps = np.array([gps[0:trim_last, 0],
                    gps[0:trim_last, 1],
                    gps[0:trim_last, 2]]).transpose()
    imu_time = imu_time[0:trim_last]
    imu = np.array([imu[0:trim_last, 0],
                    imu[0:trim_last, 1],
                    imu[0:trim_last, 2]]).transpose()

    # Interpolate odometry data
    odom_data = (odom_time, odom_pos, odom_att)
    gps_data = [gps_time, gps]
    imu_data = [imu_time, imu]
    new_odom_data = interpolate_odom_gps_imu(odom_data, gps_data, imu_data)
    interp_odom_pos = np.array(new_odom_data[0:3]).transpose()
    interp_odom_att = np.array(new_odom_data[3:6]).transpose()

    return {"gps_time": gps_time, "gps": gps, "odom_pos": interp_odom_pos,
            "imu_time": imu_time, "imu": imu, "odom_att": interp_odom_att,
            "vel_time": vel_time, "vel": vel,
            "gimbal_angles_t": gimbal_data[0], "gimbal_angles": gimbal_data[1],
            "ref_angles_t": gimbal_data[2], "ref_angles": gimbal_data[3]}


def plot_data(mono_bag_path, scc_bag_path, dcc_bag_path):
    mono_data = parse_bag(mono_bag_path,
                          trim_last=-100,
                          yaw_offset=35)
    scc_data = parse_bag(scc_bag_path,
                         trim_last=-100,
                         yaw_offset=30)
    dcc_data = parse_bag(dcc_bag_path,
                         trim_last=-100,
                         yaw_offset=35)

    # Set plot style
    plt.style.use("ggplot")

    # Figure 1 - trajectory plot
    # plot_trajectory(mono_data, scc_data, dcc_data)

    # Figure 2 - translation errors
    plot_translation_errors(mono_data, scc_data, dcc_data)
    # plt.savefig("plot_translation_errors.png")

    # Figure 3 - rotation errors
    # plot_rotation_errors(mono_data, scc_data)
    # plt.savefig("plot_rotation_errors.png")

    # Figure 4 - Gimbal angles measured vs estimated
    # plot_gimbal_angles(*gimbal_data)
    # plt.savefig("plot_gimbal_errors%s.png")

    # plt.show()


if __name__ == "__main__":
    mono_bag_path = sys.argv[1]
    scc_bag_path = sys.argv[2]
    dcc_bag_path = sys.argv[3]

    plot_data(mono_bag_path, scc_bag_path, dcc_bag_path)
