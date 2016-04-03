#ifndef __ROS_UTIL_HPP__
#define __ROS_UTIL_HPP__

#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "awesomo/camera.hpp"


// FUNCTIONS
void rotation_matrix(double phi, double theta, double psi, double *rot_mat);
void single_rotation_matrix(double psi, double *rot_mat);
void mat3_dot_vec3(double *m, double *v, double *out);
void fix_coordinate_frames(
    TagPose &pose,
	double *rot_mat,
    double *pos,
    tf::Quaternion &quat
);
void build_pose_stamped_msg(
    int seq,
    TagPose &pose,
	double *rot_mat,
    geometry_msgs::PoseStamped &pose_msg
);
void build_pose_stamped_cov_msg(
    int seq,
    TagPose &pose,
	double *rot_mat,
    geometry_msgs::PoseWithCovarianceStamped &pose_msg
);
void print_pose_stamped_msg(geometry_msgs::PoseStamped &pose_msg);
void print_pose_cov_stamped_msg(geometry_msgs::PoseWithCovarianceStamped &pose_msg);

#endif
