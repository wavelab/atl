#ifndef __AWESOMO_ROS_NODES_FAKE_MAVROS_NODE_HPP__
#define __AWESOMO_ROS_NODES_FAKE_MAVROS_NODE_HPP__

#include <ros/ros.h>
#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_fake_mavros"
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

}  // end of awesomo namespace
#endif
