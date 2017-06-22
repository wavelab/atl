#ifndef __atl_ROS_NODES_GIMBAL_NODE_HPP__
#define __atl_ROS_NODES_GIMBAL_NODE_HPP__

#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <atl_core/atl_core.hpp>

#include "atl_ros/utils/node.hpp"
#include "atl_ros/utils/msgs.hpp"

namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_gimbal"
#define NODE_RATE 40

// PUBLISH TOPICS
#define SBGC_IMU_TOPIC "/atl/sbgc/imu"
#define SBGC_RAW_ENCODER_TOPIC "atl/sbgc/encoders/rpy"
#define POSITION_TOPIC "/atl/gimbal/position/inertial"
#define FRAME_ORIENTATION_TOPIC "/atl/gimbal/frame/orientation/inertial"
#define JOINT_ORIENTATION_TOPIC "/atl/gimbal/joint/orientation/inertial"
#define ENCODER_ORIENTATION_TOPIC "atl/gimbal/joint/orientation/body"
// SUBSCRIBE TOPICS
#define QUAD_POSE_TOPIC "/atl/quadrotor/pose/local"
#define SETPOINT_TOPIC "/atl/gimbal/setpoint/attitude"
#define TRACK_TOPIC "/atl/gimbal/target/track"
#define SHUTDOWN_TOPIC "/atl/gimbal/shutdown"


class GimbalNode : public ROSNode {
public:
  std::string quad_frame;
  std::string gimbal_imu;
  Gimbal gimbal;
  Vec3 set_points;

  GimbalNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~GimbalNode(void);
  int configure(std::string node_name, int hz);
  int publishIMU(Vec3 euler);
  int publishRawEncoder(Vec3 encoder_euler);
  int publishPosition(Vec3 pos);
  int publishFrameOrientation(Quaternion q);
  int publishJointOrientation(Quaternion q);
  int publishEncoderOrientation(Quaternion q);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void setAttitudeCallback(const geometry_msgs::Vector3 &msg);
  void trackTargetCallback(const geometry_msgs::Vector3 &msg);
  int loopCallback(void);
};

}  // end of atl namespace
#endif
