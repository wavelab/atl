#ifndef __AWESOMO_ROS_GIMBAL_NODE_HPP__
#define __AWESOMO_ROS_GIMBAL_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"

namespace awesomo {

#define NODE_NAME "gimbal_node"
#define NODE_RATE 100

#define POSE_TOPIC "/mavros/local_position/pose"

class GimbalNode : public ROSNode {
public:
  bool configured;
  // Gimbal gimbal;

  GimbalNode(void);
  int configure(std::string node_name, int hz);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
