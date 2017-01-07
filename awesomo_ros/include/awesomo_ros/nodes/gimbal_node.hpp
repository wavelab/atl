#ifndef __AWESOMO_ROS_NODES_GIMBAL_NODE_HPP__
#define __AWESOMO_ROS_NODES_GIMBAL_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"

namespace awesomo {

// NODE SETTINGS
#define GIMBAL_NODE_NAME "awesomo_gimbal"
#define GIMBAL_NODE_RATE 100

// PUBLISH TOPICS

// SUBSCRIBE TOPICS


class GimbalNode : public ROSNode {
public:
  Gimbal gimbal;

  int configure(std::string node_name, int hz);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
