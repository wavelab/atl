#ifndef __atl_ROS_NODES_FAKE_MAVROS_NODE_HPP__
#define __atl_ROS_NODES_FAKE_MAVROS_NODE_HPP__

#include <ros/ros.h>
#include <atl_core/atl_core.hpp>

#include "atl_ros/utils/node.hpp"
#include "atl_ros/utils/msgs.hpp"


namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_fake_mavros"
#define NODE_RATE 100

// PUBLISH TOPICS
#define MAVROS_POSE_TOPIC "/mavros/local_position/pose"
#define MAVROS_VELOCITY_TOPIC "/mavros/local_position/velocity"

class FakeMavrosNode : public ROSNode {
public:
  FakeMavrosNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  int publishPose(void);
  int publishVelocity(void);
  int loopCallback(void);
};

}  // end of atl namespace
#endif
