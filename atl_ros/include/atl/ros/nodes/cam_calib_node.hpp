#ifndef ATL_ROS_NODES_CAM_CALIB_NODE_HPP
#define ATL_ROS_NODES_CAM_CALIB_NODE_HPP

#include <sys/stat.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include "atl/atl_core.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 60;

// SUBSCRIBE TOPICS
// clang-format off
static const std::string GIMBAL_FRAME_ORIENTATION_TOPIC = "/atl/gimbal/frame/orientation/inertial";
static const std::string GIMBAL_JOINT_ORIENTATION_TOPIC = "/atl/gimbal/joint/orientation/inertial";
static const std::string GIMBAL_ENCODER_ORIENTATION_TOPIC = "atl/gimbal/joint/orientation/body";
static const std::string SHUTDOWN_TOPIC = "/atl/camera/shutdown";
// clang-format on

namespace atl {

class CamCalibNode : public ROSNode {
public:
  std::string calib_dir;
  std::string static_camera_dir;
  std::string gimbal_camera_dir;
  std::ofstream gimbal_joint_file;
  std::ofstream gimbal_encoder_file;

  cv::Size chessboard_size;
  bool chessboard_detected[2] = {false, false};
  std::vector<cv::Point2f> corners_1;
  std::vector<cv::Point2f> corners_2;

  std::string static_camera_topic;
  std::string gimbal_camera_topic;

  int image_number = 0;
  cv::Mat static_camera_image;
  cv::Mat gimbal_camera_image;
  Quaternion gimbal_joint_orientation;
  Quaternion gimbal_joint_body_orientation;

  CamCalibNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~CamCalibNode() {
    this->gimbal_joint_file.close();
    this->gimbal_encoder_file.close();
  }

  int configure(int hz);
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr &msg, cv::Mat &img);
  void staticCameraCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalCameraCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointBodyCallback(const geometry_msgs::Quaternion &msg);
  bool chessboardDetected();
  void showImages();
  void saveImages();
  void saveGimbalMeasurements();
  int loopCallback();
};

} // namespace atl
#endif
