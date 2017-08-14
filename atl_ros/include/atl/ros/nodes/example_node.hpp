#ifndef ATL_ROS_EXAMPLE_NODE_HPP
#define ATL_ROS_EXAMPLE_NODE_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 1.0;

// PUBLISH TOPIC
static const std::string SAY_TOPIC = "/example/say";

namespace atl {

class ExampleNode : public ROSNode {
public:
  ExampleNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(int hz);
  int loopCallback();
  void sayCallback(const std_msgs::String &msg);
};
;

} // namespace atl
#endif
