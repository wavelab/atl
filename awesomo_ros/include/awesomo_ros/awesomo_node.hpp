#ifndef __AWESOMO_ROS_AWESOMO_NODE_HPP__
#define __AWESOMO_ROS_AWESOMO_NODE_HPP__

#include <iostream>
#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <awesomo_msgs/KFStats.h>
#include <awesomo_msgs/KFPlotting.h>
#include <awesomo_msgs/AprilTagPose.h>
#include <awesomo_msgs/PositionControllerStats.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"


// ROS TOPICS
#define ARM_TOPIC "/mavros/cmd/arming"
#define MODE_TOPIC "/mavros/set_mode"

#define STATE_TOPIC "/mavros/state"
#define POSE_TOPIC "/mavros/local_position/pose"
#define RADIO_TOPIC "/mavros/rc/in"
#define APRILTAG_TOPIC "/awesomo/vision/pose"

#define SETPOINT_ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define SETPOINT_THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"
#define SETPOINT_POSITION_TOPIC "/mavros/setpoint_position/local"

#define POS_CONTROLLER_STATS_TOPIC "/awesomo/position_controller/stats"
#define KF_STATS_TOPIC "/awesomo/kf_estimation/stats"
#define KF_PLOTTING_TOPIC "/awesomo/kf_estimation/states"


namespace awesomo {

class AwesomoNode : public ROSNode {
public:
  bool configured;

  Quadrotor quadrotor;

  Pose world_pose;
  mavros_msgs::State mavros_state;
  int rc_in[16];
  TagPose tag_pose;

  ros::ServiceClient mode_client;
  ros::ServiceClient arming_client;

  AwesomoNode(void);
  int configure(std::string config_path);
  void waitForConnection(void);
  int disarm(void);
  int setOffboardModeOn(void);
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void poseCallback(const geometry_msgs::PoseStamped &msg);
  void radioCallback(const mavros_msgs::RCIn &msg);
  void aprilTagCallback(const awesomo_msgs::AprilTagPose &msg);
  int awesomoLoopCallback(void);
  void publishStats(void);
};

}  // end of awesomo namespace
#endif
