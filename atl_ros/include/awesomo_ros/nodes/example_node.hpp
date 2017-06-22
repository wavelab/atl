#ifndef __atl_ROS_EXAMPLE_NODE_HPP__
#define __atl_ROS_EXAMPLE_NODE_HPP__

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <atl_core/atl_core.hpp>

#include "atl_ros/utils/node.hpp"
#include "atl_ros/utils/msgs.hpp"


namespace atl {

// NODE SETTINGS
#define NODE_NAME "example"
#define NODE_RATE 1

// PUBLISH TOPIC
#define SAY_TOPIC "/example/say"

class ExampleNode : public ROSNode {
public:
  ExampleNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  int loopCallback(void);
  void sayCallback(const std_msgs::String &msg);
};
;

}  // end of atl namespace
#endif
