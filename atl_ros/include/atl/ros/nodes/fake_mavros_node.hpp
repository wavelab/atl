#ifndef ATL_ROS_NODES_FAKE_MAVROS_NODE_HPP
#define ATL_ROS_NODES_FAKE_MAVROS_NODE_HPP

#include <atl/atl_core.hpp>
#include <ros/ros.h>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 100;

// PUBLISH TOPICS
// clang-format off
static const std::string MAVROS_POSE_TOPIC = "/mavros/local_position/pose";
static const std::string MAVROS_VELOCITY_TOPIC = "/mavros/local_position/velocity";
// clang-format on

namespace atl {

class FakeMavrosNode : public ROSNode {
public:
  FakeMavrosNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const int hz);
  int publishPose();
  int publishVelocity();
  int loopCallback();
};

} // namespace atl
#endif
