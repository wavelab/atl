#ifndef __AWESOMO_ROS_EXAMPLE_NODE_HPP__
#define __AWESOMO_ROS_EXAMPLE_NODE_HPP__

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"


namespace awesomo {

#define NODE_NAME "apriltag_node"
#define NODE_RATE 1

#define SAY_TOPIC "awesomo/apriltag/say"
#define APRILTAG_POSE_TOPIC "awesomo/apriltag/pose"

class AprilTagNode : public ROSNode {
public:
  int configure(const std::string &node_name, int hz);
  int loopCallback(void);
  void sayCallback(const std_msgs::String &msg);
};
;

}  // end of awesomo namespace
#endif
