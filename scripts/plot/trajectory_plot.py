#!/usr/bin/env python2
import csv
from math import pi
from math import pow
from math import asin
from math import atan2
from math import fabs

import rosbag
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from rosbag_parser import rosbag_parser

# bag = rosbag.Bag("../atl_data/170220-position_controller_tuning_1.bag")
# bag = rosbag.Bag("../atl_data/170220-position_controller_tuning_2.bag")
# bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/tracking_inertial_mode.bag")
# bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/tracking_blown_by_wind.bag")
# bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/tracking_better_but_losted.bag")
# bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/long_track_active_gimbal.bag")
# bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/long-track_starting_with_vear_to_left.bag")

# bag =rosbag.Bag("/home/chutsu/Dropbox/atl_bags/simulation/s_curve-radius_10-velocity_1.bag")
bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/simulation/straight_line_10.bag")
# bag = rosbag.Bag("/home/chutsu/Dropbox/atl_bags/simulation/circle-radius_12-velocity_1.bag")

traj_file = "../atl_configs/configs/trajectory/9.csv"


def figsize(scale):
    fig_width_pt = 505.89                           # Get this from LaTeX using \the\textwidth
    inches_per_pt = 1.0/72.27                       # Convert pt to inch
    golden_mean = (np.sqrt(5.0)-1.0)/2.0            # Aesthetic ratio (you could change this)
    fig_width = fig_width_pt*inches_per_pt*scale    # width in inches
    fig_height = fig_width*golden_mean              # height in inches
    fig_size = [fig_width,fig_height]
    return fig_size

def parse_trajectory(traj_file):
    # setup
    f = open(traj_file, "r")
    reader = csv.reader(f, delimiter=",")
    next(reader)  # skip header

    data = {
        "wp_pos_x": [], "wp_pos_z": [],
        "wp_vel_x": [], "wp_vel_z": [],
        "wp_thrust": [], "wp_theta": []
    }

    # parse data
    for row in reader:
        print row



def quat2euler(quat, euler_seq):
    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    qw2 = pow(qw, 2)
    qx2 = pow(qx, 2)
    qy2 = pow(qy, 2)
    qz2 = pow(qz, 2)

    if euler_seq == 123:
        phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2));
        theta = asin(2 * (qx * qz + qy * qw));
        psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2));

    elif euler_seq == 321:
        phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
        theta = asin(2 * (qy * qw - qx * qz))
        psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

    else:
        raise RuntimeError("Not implemented!")

    return (phi, theta, psi)


class PoseData:
    def __init__(self):
        self.pose_time = []
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.orientation_w = []
        self.orientation_x = []
        self.orientation_y = []
        self.orientation_z = []
        self.roll = []
        self.pitch = []
        self.yaw = []

    def parse_msg(self, ros_msg, ros_time):
        self.pose_time.append(ros_time)
        self.position_x.append(ros_msg.position.x)
        self.position_y.append(ros_msg.position.y)
        self.position_z.append(ros_msg.position.z)
        self.orientation_w.append(ros_msg.orientation.w)
        self.orientation_x.append(ros_msg.orientation.x)
        self.orientation_y.append(ros_msg.orientation.y)
        self.orientation_z.append(ros_msg.orientation.z)

        quat = [
            ros_msg.orientation.w,
            ros_msg.orientation.x,
            ros_msg.orientation.y,
            ros_msg.orientation.z
        ]
        roll, pitch, yaw = quat2euler(quat, 321)
        self.roll.append(roll * 180 / pi)
        self.pitch.append(pitch * 180 / pi)
        self.yaw.append(yaw * 180 / pi)



class PoseStampedData:
    def __init__(self):
        self.pose_time = []
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.orientation_w = []
        self.orientation_x = []
        self.orientation_y = []
        self.orientation_z = []
        self.roll = []
        self.pitch = []
        self.yaw = []

    def parse_msg(self, ros_msg, ros_time):
        self.pose_time.append(ros_time)
        self.position_x.append(ros_msg.pose.position.x)
        self.position_y.append(ros_msg.pose.position.y)
        self.position_z.append(ros_msg.pose.position.z)
        self.orientation_w.append(ros_msg.pose.orientation.w)
        self.orientation_x.append(ros_msg.pose.orientation.x)
        self.orientation_y.append(ros_msg.pose.orientation.y)
        self.orientation_z.append(ros_msg.pose.orientation.z)

        quat = [
            ros_msg.pose.orientation.w,
            ros_msg.pose.orientation.x,
            ros_msg.pose.orientation.y,
            ros_msg.pose.orientation.z
        ]
        roll, pitch, yaw = quat2euler(quat, 321)
        self.roll.append(roll * 180 / pi)
        self.pitch.append(pitch * 180 / pi)
        self.yaw.append(yaw * 180 / pi)


class TwistStampedData:
    def __init__(self):
        self.twist_time = []
        self.linear_x = []
        self.linear_y = []
        self.linear_z = []
        self.angular_x = []
        self.angular_y = []
        self.angular_z = []

    def parse_msg(self, ros_msg, ros_time):
        self.twist_time.append(ros_time)
        self.linear_x.append(ros_msg.twist.linear.x)
        self.linear_y.append(ros_msg.twist.linear.y)
        self.linear_z.append(ros_msg.twist.linear.z)
        self.angular_x.append(ros_msg.twist.angular.x)
        self.angular_y.append(ros_msg.twist.angular.y)
        self.angular_z.append(ros_msg.twist.angular.z)


class Vec3Data:
    def __init__(self):
        self.vec3_time = []
        self.x = []
        self.y = []
        self.z = []

    def parse_msg(self, ros_msg, ros_time):
        self.vec3_time.append(ros_time)
        self.x.append(ros_msg.x)
        self.y.append(ros_msg.y)
        self.z.append(ros_msg.z)


class QuadData:
    def __init__(self):
        # "/atl/quadrotor/pose/local"
        self.pose = PoseStampedData()

        # "/atl/quadrotor/velocity/local"
        self.velocity = TwistStampedData()

    def parse_pose(self, ros_msg, ros_time):
        self.pose.parse_msg(ros_msg, ros_time)

    def parse_velocity(self, ros_msg, ros_time):
        self.velocity.parse_msg(ros_msg, ros_time)


class LZData:
    def __init__(self):
        # "/prototype/lz/pose"
        self.pose = PoseData()

    def parse_pose(self, ros_msg, ros_time):
        self.pose.parse_msg(ros_msg, ros_time)


class TagData:
    def __init__(self):
        # "/atl/apriltag/target/position/inertial"
        self.position_if = Vec3Data()

        # "/atl/apriltag/target/position/body"
        self.position_bf = Vec3Data()

    def parse_inertial(self, ros_msg, ros_time):
        self.position_if.parse_msg(ros_msg, ros_time)

    def parse_body(self, ros_msg, ros_time):
        self.position_bf.parse_msg(ros_msg, ros_time)


class EstimateData:
    def __init__(self):
        # "/atl/estimate/landing_target/position/inertial"
        self.position_if = Vec3Data()

        # "/atl/estimate/landing_target/position/body"
        self.position_bf = Vec3Data()

        # "/atl/estimate/landing_target/velocity/inertal"
        self.vel_if = Vec3Data()

        # "/atl/estimate/landing_target/velocity/body"
        self.vel_bf = Vec3Data()

    def parse_position_inertial(self, ros_msg, ros_time):
        self.position_if.parse_msg(ros_msg, ros_time)

    def parse_position_body(self, ros_msg, ros_time):
        self.position_bf.parse_msg(ros_msg, ros_time)

    def parse_velocity_inertial(self, ros_msg, ros_time):
        self.vel_if.parse_msg(ros_msg, ros_time)

    def parse_velocity_body(self, ros_msg, ros_time):
        self.vel_bf.parse_msg(ros_msg, ros_time)


def plot_quad_tag_xy(quad_data, tag_data):
    plt.figure()
    plt.plot(quad_data.pose.position_x,
             quad_data.pose.position_y,
             label="quadrotor")
    plt.plot(tag_data.position_if.x,
             tag_data.position_if.y,
             label="apriltag")
    plt.title("x vs y")
    plt.legend(loc=0)
    plt.xlabel("x")
    plt.ylabel("y")


def plot_quad_tag_xyz(quad_data, tag_data):
    plt.figure()

    plt.subplot(311)
    plt.plot(quad_data.pose.pose_time,
             quad_data.pose.position_x,
             label="quadrotor")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.x,
             label="apriltag")
    plt.title("t vs x")
    plt.xlabel("t")
    plt.ylabel("x")

    plt.subplot(312)
    plt.plot(quad_data.pose.pose_time,
             quad_data.pose.position_y,
             label="quadrotor")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.y,
             label="apriltag")
    plt.title("t vs y")
    plt.xlabel("t")
    plt.ylabel("y")

    plt.subplot(313)
    plt.plot(quad_data.pose.pose_time,
             quad_data.pose.position_z,
             label="quadrotor")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.z,
             label="apriltag")
    plt.title("t vs z")
    plt.xlabel("t")
    plt.ylabel("z")


def plot_quad_tag_3d(quad_data, tag_data, lz_data):
    fig = plt.figure(figsize=figsize(1.0))

    ax = fig.add_subplot(111, projection='3d')

    ax.plot(quad_data.pose.position_x,
            quad_data.pose.position_y,
            quad_data.pose.position_z,
            label="Quadrotor")

    ax.plot(tag_data.position_if.x,
            tag_data.position_if.y,
            tag_data.position_if.z,
            label="Measured AprilTag")

    if lz_data is not None:
        ax.plot(lz_data.pose.position_x,
                lz_data.pose.position_y,
                lz_data.pose.position_z,
                label="Landing Target")

    # limits
    ax.set_xlim(-12.5, 12.5)
    ax.set_ylim(0.0, 25.0)
    ax.set_zlim(0.0, 6.0)

    # labels
    ax.set_xlabel("x-position (m)")
    ax.set_ylabel("y-position (m)")
    ax.set_zlabel("z-position (m)")

    # text
    ax.text(0, 0, 5, "Start")
    ax.text(-6, 10, 5, "Tracking")
    ax.text(-6, 25, 5, "Landing")
    ax.text(-4, 20, 0, "End")

    ax.legend()

    # fig = plt.figure()
    # ax = fig.add_subplot(211)
    # ax.plot(quad_data.pose.pose_time, quad_data.pose.pitch)
    # ax.set_xlabel("time (s)")
    # ax.set_ylabel("pitch angle (deg)")
    #
    # ax = fig.add_subplot(212)
    # ax.plot(quad_data.pose.pose_time, quad_data.velocity.linear_x, label="vx")
    # ax.plot(quad_data.pose.pose_time, quad_data.velocity.linear_y, label="vy")
    # ax.plot(quad_data.pose.pose_time, quad_data.velocity.linear_z, label="vz")
    # ax.set_xlabel("time (s)")
    # ax.set_ylabel("velocity (ms^-1)")
    # ax.legend()

def relative_data(quad_data, lz_data):
    rel_time = []
    rel_pos_x = []
    rel_pos_y = []
    rel_pos_z = []

    lz_pos_x = []
    lz_pos_y = []
    lz_pos_z = []

    quad_time_index = 0
    for i in range(len(lz_data.pose.pose_time)):
        if (quad_time_index + 1) == len(quad_data.pose.pose_time):
            break

        dt = fabs(lz_data.pose.pose_time[i] - quad_data.pose.pose_time[quad_time_index])

        if dt < 0.005:
            rel_time.append(quad_data.pose.pose_time[quad_time_index])
            lz_pos_x.append(lz_data.pose.position_x[i])
            lz_pos_y.append(lz_data.pose.position_y[i])
            lz_pos_z.append(lz_data.pose.position_z[i])
            quad_time_index += 1

    for i in range(len(lz_pos_x)):
        rel_pos_x.append(lz_pos_x[i] - quad_data.pose.position_x[i])
        rel_pos_y.append(lz_pos_y[i] - quad_data.pose.position_y[i] - 0.13)
        rel_pos_z.append(lz_pos_z[i] - quad_data.pose.position_z[i])

    return (rel_pos_x, rel_pos_y, rel_pos_z, rel_time)


def align_estimated_with_ground(rel_data, est_data):
    est_time = []
    est_pos_x = []
    est_pos_y = []
    est_pos_z = []

    (rel_pos_x, rel_pos_y, rel_pos_z, rel_time) = rel_data

    t_index = 0
    for i in range(len(est_data.position_bf.vec3_time)):
        if (t_index + 1) == len(rel_time):
            break

        dt = fabs(est_data.position_bf.vec3_time[i] - rel_time[t_index])
        if dt < 0.01:
            est_time.append(est_data.position_bf.x[i])
            est_pos_x.append(est_data.position_bf.x[i])
            est_pos_y.append(est_data.position_bf.y[i])
            est_pos_z.append(est_data.position_bf.z[i])
            t_index += 1

    # print len(rel_time)
    print len(rel_time)
    print len(est_data.position_bf.vec3_time)
    print len(est_pos_x)


def rms_errors(quad_data, est_data, tag_data):
    rel_data = relative_data(quad_data, lz_data)
    (rel_pos_x, rel_pos_y, rel_pos_z, rel_time) = rel_data

    # rel_file = open("rel_data.csv", "w")
    # est_file = open("est_data.csv", "w")
    #
    # est_file.write("time, x, y, z\n")
    # for i in range(len(est_data.position_bf.vec3_time)):
    #     est_file.write("{0}, {1}, {2}, {3}\n".format(
    #         est_data.position_bf.vec3_time[i],
    #         est_data.position_bf.x[i],
    #         est_data.position_bf.y[i],
    #         est_data.position_bf.z[i]
    #     ))
    # est_file.close()
    #
    # rel_file.write("time, x, y, z\n")
    # for i in range(len(rel_time)):
    #     rel_file.write("{0}, {1}, {2}, {3}\n".format(
    #         rel_time[i],
    #         rel_pos_x[i],
    #         rel_pos_y[i],
    #         rel_pos_z[i]
    #     ))
    # rel_file.close()


def plot_est_tag_xyz(est_data, tag_data, quad_data, lz_data):
    est_time_end = est_data.position_bf.vec3_time[-1]
    tag_time_end = tag_data.position_bf.vec3_time[-1]
    time_start = 12.0
    time_end = 29.0

    (rel_pos_x, rel_pos_y, rel_pos_z, rel_time) = relative_data(quad_data, lz_data)

    # plt.figure(figsize=(7.0, 7.0))
    #
    # plt.subplot(311)
    # plt.tight_layout()
    # plt.plot(est_data.position_bf.vec3_time,
    #          est_data.position_bf.x,
    #          label="Estimate")
    # plt.plot(rel_time,
    #          rel_pos_y,
    #          label="Ground Truth")
    # plt.plot(tag_data.position_bf.vec3_time,
    #          tag_data.position_bf.x,
    #          label="Measured")
    # plt.title("Estimated Target Relative Position in Body X")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Position (m)")
    # plt.xlim(time_start, time_end)
    # plt.ylim(-1.5, 1.5)
    # plt.legend(loc=0)
    #
    # plt.subplot(312)
    # plt.tight_layout()
    # plt.plot(est_data.position_bf.vec3_time,
    #          est_data.position_bf.y,
    #          label="Estimate")
    # plt.plot(rel_time,
    #          rel_pos_x,
    #          label="Ground Truth")
    # plt.plot(tag_data.position_bf.vec3_time,
    #          tag_data.position_bf.y,
    #          label="Measured")
    # plt.title("Estimated Target Relative Position in Body Y")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Position (m)")
    # plt.xlim(time_start, time_end)
    # plt.ylim(-1.5, 1.5)
    # plt.legend(loc=0)
    #
    # plt.subplot(313)
    # plt.tight_layout()
    # plt.plot(est_data.position_bf.vec3_time,
    #          est_data.position_bf.z,
    #          label="Estimate")
    # plt.plot(rel_time,
    #          rel_pos_z,
    #          label="Ground Truth")
    # plt.plot(tag_data.position_bf.vec3_time,
    #          tag_data.position_bf.z,
    #          label="Measured")
    # plt.title("Estimated Target Relative Position in Body Z")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Position (m)")
    # plt.xlim(time_start, time_end)
    # plt.ylim(-6.0, 0.0)
    # plt.legend(loc=0)
    #
    # plt.subplots_adjust(bottom=0.08, hspace=0.9)


if __name__ == "__main__":
    quad_data = QuadData()
    tag_data = TagData()
    est_data = EstimateData()
    lz_data = LZData()

    params = [
        {"topic": "/atl/quadrotor/pose/local",
         "callback": quad_data.parse_pose},
        {"topic": "/atl/quadrotor/velocity/local",
            "callback": quad_data.parse_velocity},
        {"topic": "/atl/apriltag/target/position/inertial",
            "callback": tag_data.parse_inertial},
        {"topic": "/atl/apriltag/target/position/body",
            "callback": tag_data.parse_body},
        # {"topic": "/atl/estimate/landing_target/position/inertial",
        #     "callback": est_data.parse_position_inertial},
        {"topic": "/atl/estimate/landing_target/position/body",
            "callback": est_data.parse_position_body},
        # {"topic": "/atl/estimate/landing_target/velocity/inertial",
        #     "callback": est_data.parse_velocity_inertial},
        {"topic": "/atl/estimate/landing_target/velocity/body",
            "callback": est_data.parse_velocity_body},
        {"topic": "/atl/lz/pose",
            "callback": lz_data.parse_pose}
    ]

    # parse_trajectory(traj_file)

    # # rosbag_parser(bag, params)
    rosbag_parser(bag, params, 34.9, 6.0)

    # relative_data(quad_data, lz_data)
    rms_errors(quad_data, est_data, tag_data)

    # plt.style.use("ggplot")
    # # # plot_quad_tag_xy(quad_data, tag_data)
    # # # plot_quad_tag_xyz(quad_data, tag_data)
    # # plot_quad_tag_3d(quad_data, tag_data, lz_data)
    # plot_est_tag_xyz(est_data, tag_data, quad_data, lz_data)
    # plt.savefig("estimation.png")
    # plt.show()
