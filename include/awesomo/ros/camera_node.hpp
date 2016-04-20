#ifndef __CAMERA_NODE_HPP__
#define __CAMERA_NODE_HPP__

#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "awesomo/util.hpp"
#include "awesomo/camera.hpp"
#include "awesomo/ros/ros_util.hpp"


// DEFINES
#define ROS_TOPIC "awesomo/apriltag_pose"
#define ROS_TOPIC2 "awesomo/test"
// #define ROS_TOPIC "mavros/vision_pose/pose"
// #define ROS_TOPIC "mavros/mocap/pose"
// #define ROS_TOPIC "mavros/vision_pose/pose_cov"


// #define FIREFLY_640 "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_640.yaml"
// #define FIREFLY_320 "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_320.yaml"
// #define FIREFLY_160 "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_160.yaml"

// #define FIREFLY_640 "/home/stan/Projects/awesomo/configs/pointgrey_firefly/ost_640.yaml"
// #define FIREFLY_320 "/home/stan/Projects/awesomo/configs/pointgrey_firefly/ost_320.yaml"
// #define FIREFLY_160 "/home/stan/Projects/awesomo/configs/pointgrey_firefly/ost_160.yaml"

// #define FIREFLY_640 "/home/odroid/awesomo/configs/pointgrey_firefly/ost_640.yaml"
// #define FIREFLY_320 "/home/odroid/awesomo/configs/pointgrey_firefly/ost_320.yaml"
// #define FIREFLY_160 "/home/odroid/awesomo/configs/pointgrey_firefly/ost_160.yaml"



#endif
