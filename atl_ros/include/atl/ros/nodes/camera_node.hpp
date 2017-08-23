#ifndef ATL_ROS_NODES_CAMERA_NODE_HPP
#define ATL_ROS_NODES_CAMERA_NODE_HPP

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 100;

// PUBLISH TOPICS
static const std::string CAMERA_IMAGE_TOPIC = "/atl/camera/image";

// SUBSCRIBE TOPICS
// clang-format off
static const std::string GIMBAL_POSITION_TOPIC = "/atl/quadrotor/pose/local";
static const std::string GIMBAL_FRAME_ORIENTATION_TOPIC = "/atl/gimbal/frame/orientation/inertial";
static const std::string GIMBAL_JOINT_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/inertial";
static const std::string APRILTAG_TOPIC = "/atl/apriltag/target";
static const std::string SHUTDOWN_TOPIC = "/atl/camera/shutdown";
// clang-format on

namespace atl {

class CameraNode : public ROSNode {
public:
  Camera camera;
  cv::Mat image;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Vec3 gimbal_position;
  TagPose tag;

  CameraNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(const int hz);

  int publishImage();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalPositionCallback(const geometry_msgs::Vector3 &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void aprilTagCallback(const atl_msgs::AprilTagPose &msg);
  int loopCallback();
};

} // namespace atl
#endif
