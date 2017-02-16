#ifndef __AWESOMO_ROS_NODES_IMU_NODE_HPP__
#define __AWESOMO_ROS_NODES_IMU_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_imu"
#define NODE_RATE 100

// PUBLISH TOPICS
#define IMU_TOPIC "/awesomo/imu"

class IMUNode : public ROSNode {
public:
  MPU6050 imu;

  IMUNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  int publishData(void);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
