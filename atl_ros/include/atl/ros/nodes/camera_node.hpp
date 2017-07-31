#ifndef ATL_ROS_NODES_CAMERA_NODE_HPP
#define ATL_ROS_NODES_CAMERA_NODE_HPP

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_camera"
#define NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMAGE_TOPIC "/atl/camera/image"

// SUBSCRIBE TOPICS
// clang-format off
#define APRILTAG_TOPIC "/atl/apriltag/target"
#define GIMBAL_POSITION_TOPIC "/atl/quadrotor/pose/local"
#define GIMBAL_FRAME_ORIENTATION_TOPIC "/atl/gimbal/frame/orientation/inertial"
#define GIMBAL_JOINT_ORIENTATION_TOPIC "/atl/gimbal/joint/orientation/inertial"
#define APRILTAG_TOPIC "/atl/apriltag/target"
#define SHUTDOWN_TOPIC "/atl/camera/shutdown"
// clang-format on

class CameraNode : public ROSNode {
public:
  Camera camera;
  cv::Mat image;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Vec3 gimbal_position;
  TagPose tag;

  CameraNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  int publishImage();
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalPositionCallback(const geometry_msgs::Vector3 &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void aprilTagCallback(const atl_msgs::AprilTagPose &msg);
  int loopCallback();
};

}  // namespace atl
#endif
