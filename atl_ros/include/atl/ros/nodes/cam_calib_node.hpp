#ifndef ATL_ROS_NODES_CAM_CALIB_NODE_HPP
#define ATL_ROS_NODES_CAM_CALIB_NODE_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include "atl/atl_core.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_cam_calib"
#define NODE_RATE 60

// PUBLISH TOPICS
#define CAMERA_IMAGE_TOPIC "/atl/camera/image"

// SUBSCRIBE TOPICS
// clang-format off
#define GIMBAL_FRAME_ORIENTATION_TOPIC "/atl/gimbal/frame/orientation/inertial"
#define GIMBAL_JOINT_ORIENTATION_TOPIC "/atl/gimbal/joint/orientation/inertial"
#define GIMBAL_ENCODER_ORIENTATION_TOPIC "atl/gimbal/joint/orientation/body"
// clang-format on

#define SHUTDOWN_TOPIC "/atl/camera/shutdown"

class CamCalibNode : public ROSNode {
public:
  DC1394Camera camera;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Quaternion gimbal_joint_body_orientation;

  CamCalibNode(int argc, char **argv) : ROSNode(argc, argv) {}

  int configure(const std::string &node_name, int hz);
  int publishImage(cv::Mat &image);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointBodyCallback(const geometry_msgs::Quaternion &msg);
  int loopCallback();
};

}  // namespace atl
#endif
