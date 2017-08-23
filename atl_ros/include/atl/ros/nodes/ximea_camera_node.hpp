#ifndef ATL_ROS_NODES_XIMEA_CAMERA_NODE_HPP
#define ATL_ROS_NODES_XIMEA_CAMERA_NODE_HPP

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 100;

// clang-format off
// PUBLISH TOPICS
static const std::string CAMERA_IMAGE_TOPIC = "/atl/camera/image";

// SUBSCRIBE TOPICS
static const std::string GIMBAL_FRAME_ORIENTATION_TOPIC = "/atl/gimbal/frame/orientation/inertial";
static const std::string GIMBAL_JOINT_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/inertial";
static const std::string APRILTAG_TOPIC = "/atl/apriltag/target";
static const std::string SHUTDOWN_TOPIC = "/atl/camera/shutdown";
// clang-format on

namespace atl {

class XimeaCameraNode : public ROSNode {
public:
  XimeaCamera camera;
  cv::Mat image;
  bool grey_scale;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Vec3 gimbal_position;
  TagPose tag;

  XimeaCameraNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const int hz);
  int publishImage();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void aprilTagCallback(const atl_msgs::AprilTagPose &msg);
  int loopCallback();
};

} // namespace atl
#endif
