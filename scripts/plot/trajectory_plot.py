#!/usr/bin/env python2
import rosbag
from rosbag_parser import parse_pose_stamped
from rosbag_plotter import plot_vec3_vs_time

# bag = rosbag.Bag("awesomo_bags/170220-position_controller_tuning_1.bag")
bag = rosbag.Bag("awesomo_bags/170220-position_controller_tuning_2.bag")

topic = "/awesomo/quadrotor/pose/local"
position_data = parse_pose_stamped(bag, topic)
plot_vec3_vs_time(position_data)
