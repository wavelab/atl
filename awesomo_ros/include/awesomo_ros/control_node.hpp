#ifndef __AWESOMO_ROS_CONTROL_NODE_HPP__
#define __AWESOMO_ROS_CONTROL_NODE_HPP__

#include <ros/ros.h>

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
#include <awesomo_msgs/PositionControllerSettings.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"


namespace awesomo {

// ROS NODE
#define CONTROL_NODE_NAME "awesomo_control"
#define CONTROL_NODE_RATE 100

class ControlNode : public ROSNode {
public:
  bool configured;

  Quadrotor quadrotor;

  Pose world_pose;
  mavros_msgs::State mavros_state;
  int rc_in[16];
  TagPose tag_pose;

  ros::ServiceClient mode_client;
  ros::ServiceClient arming_client;

  ControlNode(void);
  int configure(std::string node_name, int hz);
  void waitForConnection(void);
  int disarm(void);
  int setOffboardModeOn(void);
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void poseCallback(const geometry_msgs::PoseStamped &msg);
  void radioCallback(const mavros_msgs::RCIn &msg);
  void aprilTagCallback(const awesomo_msgs::AprilTagPose &msg);
  void hoverSetCallback(const geometry_msgs::Vector3 &msg);
  void positionControllerSetCallback(const awesomo_msgs::PositionControllerSettings &msg);
  int loopCallback(void);
  void publishStats(void);
};

}  // end of awesomo namespace
#endif
