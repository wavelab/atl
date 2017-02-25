#!/usr/bin/env python2
import rosbag
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from rosbag_parser import rosbag_parser

# bag = rosbag.Bag("../awesomo_data/170220-position_controller_tuning_1.bag")
# bag = rosbag.Bag("../awesomo_data/170220-position_controller_tuning_2.bag")
bag = rosbag.Bag("/home/chutsu/Dropbox/awesomo_bags/tracking_inertial_mode.bag")


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

    def parse_msg(self, ros_msg, ros_time):
        self.pose_time.append(ros_time)
        self.position_x.append(ros_msg.pose.position.x)
        self.position_y.append(ros_msg.pose.position.y)
        self.position_z.append(ros_msg.pose.position.z)
        self.orientation_w.append(ros_msg.pose.orientation.w)
        self.orientation_x.append(ros_msg.pose.orientation.x)
        self.orientation_y.append(ros_msg.pose.orientation.y)
        self.orientation_z.append(ros_msg.pose.orientation.z)


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
        # "/awesomo/quadrotor/pose/local"
        self.pose = PoseStampedData()

        # "/awesomo/quadrotor/velocity/local"
        self.velocity = TwistStampedData()

    def parse_pose(self, ros_msg, ros_time):
        self.pose.parse_msg(ros_msg, ros_time)

    def parse_velocity(self, ros_msg, ros_time):
        self.velocity.parse_msg(ros_msg, ros_time)


class TagData:
    def __init__(self):
        # "/awesomo/apriltag/target/position/inertial"
        self.position_if = Vec3Data()

        # "/awesomo/apriltag/target/position/body"
        self.position_bf = Vec3Data()

    def parse_inertial(self, ros_msg, ros_time):
        self.position_if.parse_msg(ros_msg, ros_time)

    def parse_body(self, ros_msg, ros_time):
        self.position_bf.parse_msg(ros_msg, ros_time)


class EstimateData:
    def __init__(self):
        # "/awesomo/estimate/landing_target/position/inertial"
        self.position_if = Vec3Data()

        # "/awesomo/estimate/landing_target/position/body"
        self.position_bf = Vec3Data()

        # "/awesomo/estimate/landing_target/velocity/inertal"
        self.vel_if = Vec3Data()

        # "/awesomo/estimate/landing_target/velocity/body"
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
    plt.tight_layout()
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
    plt.tight_layout()
    plt.plot(quad_data.pose.pose_time,
             quad_data.pose.position_z,
             label="quadrotor")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.z,
             label="apriltag")
    plt.title("t vs z")
    plt.xlabel("t")
    plt.ylabel("z")


def plot_quad_tag_3d(quad_data, tag_data):
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')
    ax.plot(quad_data.pose.position_x,
            quad_data.pose.position_y,
            quad_data.pose.position_z,
            label="quadrotor")
    ax.plot(tag_data.position_if.x,
            tag_data.position_if.y,
            tag_data.position_if.z,
            label="apriltag")
    ax.legend()


def plot_est_tag_xyz(est_data, tag_data):
    plt.figure()

    plt.subplot(311)
    plt.plot(est_data.position_if.vec3_time,
             est_data.position_if.x,
             label="estimate")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.x,
             label="apriltag")
    plt.title("t vs x")
    plt.xlabel("t")
    plt.ylabel("x")
    plt.legend(loc=1)

    plt.subplot(312)
    plt.tight_layout()
    plt.plot(est_data.position_if.vec3_time,
             est_data.position_if.y,
             label="estimate")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.y,
             label="apriltag")
    plt.title("t vs y")
    plt.xlabel("t")
    plt.ylabel("y")
    plt.legend(loc=1)

    plt.subplot(313)
    plt.tight_layout()
    plt.plot(est_data.position_if.vec3_time,
             est_data.position_if.z,
             label="estimate")
    plt.plot(tag_data.position_if.vec3_time,
             tag_data.position_if.z,
             label="apriltag")
    plt.title("t vs z")
    plt.xlabel("t")
    plt.ylabel("z")
    plt.legend(loc=1)


if __name__ == "__main__":
    quad_data = QuadData()
    tag_data = TagData()
    est_data = EstimateData()

    params = [
        {"topic": "/awesomo/quadrotor/pose/local",
         "callback": quad_data.parse_pose},
        {"topic": "/awesomo/quadrotor/velocity/local",
            "callback": quad_data.parse_velocity},
        {"topic": "/awesomo/apriltag/target/position/inertial",
            "callback": tag_data.parse_inertial},
        {"topic": "/awesomo/apriltag/target/position/body",
            "callback": tag_data.parse_body},
        {"topic": "/awesomo/estimate/landing_target/position/inertial",
            "callback": est_data.parse_position_inertial},
        {"topic": "/awesomo/estimate/landing_target/position/body",
            "callback": est_data.parse_position_body},
        {"topic": "/awesomo/estimate/landing_target/velocity/inertial",
            "callback": est_data.parse_velocity_inertial},
        {"topic": "/awesomo/estimate/landing_target/velocity/body",
            "callback": est_data.parse_velocity_body}
    ]
    rosbag_parser(bag, params)

    plt.style.use("ggplot")
    # plot_quad_tag_xy(quad_data, tag_data)
    # plot_quad_tag_xyz(quad_data, tag_data)
    # plot_quad_tag_3d(quad_data, tag_data)
    plot_est_tag_xyz(est_data, tag_data)
    plt.show()
