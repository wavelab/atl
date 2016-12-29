#ifndef __AWESOMO_ROS_GIMBAL_NODE_HPP__
#define __AWESOMO_ROS_GIMBAL_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"

namespace awesomo {

#define NODE_NAME "gimbal_node"
#define NODE_RATE 100

#define GIMBAL_POSE_TOPIC "/awesomo/gimbal/pose"
#define GIMBAL_SETPOINT_ANGLE_TOPIC "/awesomo/gimbal/setpoint/angle"

class GimbalNode : public ROSNode {
public:
  bool configured;
  Gimbal gimbal;

  GimbalNode(void);
  int configure(std::string node_name, int hz);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
