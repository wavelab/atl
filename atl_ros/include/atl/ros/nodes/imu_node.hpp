#ifndef ATL_ROS_NODES_IMU_NODE_HPP
#define ATL_ROS_NODES_IMU_NODE_HPP

#include <ros/ros.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 200;

// clang-format off
// PUBLISH TOPICS
static const std::string IMU_TOPIC = "/atl/imu";
static const std::string JOINT_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/inertial";
// clang-format on

namespace atl {

class IMUNode : public ROSNode {
public:
  MPU6050 imu;
  std::string quad_frame;
  std::string gimbal_imu;

  IMUNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const int hz);
  int publishIMU(const Vec3 &euler);
  int publishJointOrientation(const Quaternion &q);
  int loopCallback();
};

} // namespace atl
#endif
