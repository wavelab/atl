#ifndef __AWESOMO_ROS_EXAMPLE_NODE_HPP__
#define __AWESOMO_ROS_EXAMPLE_NODE_HPP__

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

#define NODE_NAME "example"
#define NODE_RATE 1

#define SAY_TOPIC "example_node/say"

class ExampleNode : public ROSNode {
public:
  int configure(const std::string &node_name, int hz);
  int loopCallback(void);
  void sayCallback(const std_msgs::String &msg);
};
;

}  // end of awesomo namespace
#endif
