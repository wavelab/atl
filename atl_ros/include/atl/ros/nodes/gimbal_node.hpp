#ifndef ATL_ROS_NODES_GIMBAL_NODE_HPP
#define ATL_ROS_NODES_GIMBAL_NODE_HPP

#include <unistd.h>

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 30;

// clang-format off
// PUBLISH TOPICS
static const std::string SBGC_IMU_TOPIC = "/atl/sbgc/imu";
static const std::string SBGC_RAW_ENCODER_TOPIC = "atl/sbgc/encoders/rpy";
static const std::string POSITION_TOPIC = "/atl/gimbal/position/inertial";
static const std::string FRAME_ORIENTATION_TOPIC = "/atl/gimbal/frame/orientation/inertial";
static const std::string JOINT_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/inertial";
static const std::string ENCODER_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/body";

// SUBSCRIBE TOPICS
static const std::string ACTIVATE_TOPIC = "/atl/gimbal/activate";
static const std::string QUAD_POSE_TOPIC = "/atl/quadrotor/pose/local";
static const std::string SETPOINT_TOPIC = "/atl/gimbal/setpoint/attitude";
static const std::string TRACK_TOPIC = "/atl/gimbal/target/track";
static const std::string SHUTDOWN_TOPIC = "/atl/gimbal/shutdown";
// clang-format on

namespace atl {

class GimbalNode : public ROSNode {
public:
  std::string quad_frame;
  std::string gimbal_imu;
  Gimbal gimbal;
  Vec3 set_points;

  Vec3 encoder_rpy;
  Vec3 imu_rpy;

  GimbalNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~GimbalNode();
  int configure(const int hz);
  int publishIMU(Vec3 euler);
  int publishRawEncoder(Vec3 encoder_euler);
  int publishPosition(Vec3 pos);
  int publishFrameOrientation(Quaternion q);
  int publishJointOrientation(Quaternion q);
  int publishEncoderOrientation(Quaternion q);
  void activateCallback(const std_msgs::Bool &msg);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void setAttitudeCallback(const geometry_msgs::Vector3 &msg);
  void trackTargetCallback(const geometry_msgs::Vector3 &msg);
  int loopCallback();
};

} // namespace atl
#endif
