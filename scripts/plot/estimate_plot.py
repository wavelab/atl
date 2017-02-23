#!/usr/bin/env python2
import rosbag
from rosbag_parser import parse_vec3
from rosbag_parser import parse_pose_stamped
from rosbag_parser import parse_quaternion
from rosbag_plotter import plot_vec3_vs_time
from rosbag_plotter import plot_quaternion_vs_time

bag = rosbag.Bag("/data/ros_bags/170222-estimator_debug_1.bag")
# bag = rosbag.Bag("/data/ros_bags/170222-estimator_debug_1.bag")

topic = "/awesomo/quadrotor/pose/local"
pose_data = parse_pose_stamped(bag, topic)
plot_vec3_vs_time(pose_data)

# topic = "/awesomo/estimate/landing_target/position/inertial"
# pos_if_data = parse_vec3(bag, topic)
# plot_vec3_vs_time(pos_if_data)
#
# topic = "/awesomo/estimate/landing_target/position/body"
# pos_bf_data = parse_vec3(bag, topic)
# plot_vec3_vs_time(pos_bf_data)

# topic = "/awesomo/apriltag/target/position/inertial"
# tag_if_data = parse_vec3(bag, topic)
# plot_vec3_vs_time(tag_if_data)

# topic = "/awesomo/apriltag/target/position/body"
# tag_if_data = parse_vec3(bag, topic)
# plot_vec3_vs_time(tag_if_data)

# topic = "/awesomo/imu"
# imu_data = parse_vec3(bag, topic)
# plot_vec3_vs_time(imu_data)

# topic = "/awesomo/gimbal/frame/orientation/inertial"
# gimbal_frame_data = parse_quaternion(bag, topic)
# plot_quaternion_vs_time(gimbal_frame_data)

# topic = "/awesomo/gimbal/joint/orientation/inertial"
# gimbal_joint_data = parse_quaternion(bag, topic)
# plot_quaternion_vs_time(gimbal_joint_data)

# topic = "/awesomo/gimbal/setpoint/attitude"
# gimbal_setpoint_data = parse_vec3(bag, topic)
# plot_vec3_vs_time(gimbal_setpoint_data)
