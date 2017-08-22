#ifndef ATL_ROS_NODES_PG_CAMERA_NODE_HPP
#define ATL_ROS_NODES_PG_CAMERA_NODE_HPP

#include <sys/stat.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <dji_sdk/dji_drone.h>

#include "atl/atl_core.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 61;

// SUBSCRIBE TOPICS
// clang-format off
static const std::string APRILTAG_TOPIC = "/atl/apriltag/target";
static const std::string GIMBAL_POSITION_TOPIC = "/atl/gimbal/position/inertial";
static const std::string GIMBAL_FRAME_ORIENTATION_TOPIC = "/atl/gimbal/frame/orientation/inertial";
static const std::string GIMBAL_JOINT_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/inertial";
static const std::string ENCODER_ORIENTATION_TOPIC = "atl/gimbal/joint/orientation/body";
static const std::string LT_BODY_POSITION_TOPIC = "/atl/estimate/landing_target/position/body";

static const std::string QUAD_POSITION_TOPIC = "/dji_sdk/local_position";
static const std::string QUAD_ORIENTATION_TOPIC = "/dji_sdk/attitude_quaternion";

static const std::string LT_DETECTED_TOPIC = "/atl/estimate/landing_target/detected";
static const std::string SHUTDOWN_TOPIC = "/atl/camera/shutdown";
// clang-format on

namespace atl {

class PGCameraNode : public ROSNode {
public:
  // PointGreyCamera camera;
  DC1394Camera camera;
  bool stamp_image = false;
  uint64_t guid = 0;
  std::string image_topic;

  cv::Mat image;
  Vec3 gimbal_position;
  Vec3 quadrotor_position;
  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Quaternion gimbal_joint_body_orientation;
  Quaternion quadrotor_orientation;

  bool target_detected;
  Vec3 target_pos_bf;

  PGCameraNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->gimbal_position = Vec3();
    this->gimbal_frame_orientation = Quaternion();
    this->gimbal_joint_orientation = Quaternion();

    this->target_detected = false;
    this->target_pos_bf = Vec3();
  }

  int configure(int hz);
  int publishImage();
  void gimbalPositionCallback(const geometry_msgs::Vector3 &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointBodyCallback(const geometry_msgs::Quaternion &msg);
  void targetPositionCallback(const geometry_msgs::Vector3 &msg);
  void targetDetectedCallback(const std_msgs::Bool &msg);
  void quadPositionCallback(const dji_sdk::LocalPosition &msg);
  void quadOrientationCallback(const dji_sdk::AttitudeQuaternion &msg);
  int loopCallback();
};

} // namespace atl
#endif
