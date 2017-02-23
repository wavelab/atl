#ifndef __AWESOMO_ROS_NODES_IMU_NODE_HPP__
#define __AWESOMO_ROS_NODES_IMU_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_imu"
#define NODE_RATE 200

// PUBLISH TOPICS
#define IMU_TOPIC "/awesomo/imu"
#define JOINT_ORIENTATION_TOPIC "/awesomo/gimbal/joint/orientation/inertial"

class IMUNode : public ROSNode {
public:
  MPU6050 imu;
  std::string quad_frame;
  std::string gimbal_imu;

  IMUNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  int publishIMU(Vec3 euler);
  int publishJointOrientation(Quaternion q);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
