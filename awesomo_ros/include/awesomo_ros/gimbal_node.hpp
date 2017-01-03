#ifndef __AWESOMO_ROS_GIMBAL_NODE_HPP__
#define __AWESOMO_ROS_GIMBAL_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"

namespace awesomo {

#define GIMBAL_NODE_NAME "gimbal_node"
#define GIMBAL_NODE_RATE 100

class GimbalNode : public ROSNode {
public:
  Gimbal gimbal;

  int configure(std::string node_name, int hz);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
