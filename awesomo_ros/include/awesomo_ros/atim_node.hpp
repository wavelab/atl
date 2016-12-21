#ifndef __CAMERA_NODE_HPP__
#define __CAMERA_NODE_HPP__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <awesomo_msgs/TagPoseStamped.h>

#include "awesomo_core/vision/util.hpp"
#include "awesomo_core/vision/camera.hpp"
#include "awesomo_core/vision/ros/ros_util.hpp"


// DEFINES
#define ROS_TOPIC "atim/pose"
#define POSE_TOPIC "/mavros/local_position/pose"
#define TARGET_POSE_TOPIC "awesomo/landing_target/pose"

#endif
