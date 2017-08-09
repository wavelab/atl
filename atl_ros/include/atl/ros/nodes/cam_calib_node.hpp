#ifndef ATL_ROS_NODES_CAM_CALIB_NODE_HPP
#define ATL_ROS_NODES_CAM_CALIB_NODE_HPP

#include <sys/stat.h>

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
#define NODE_RATE 100

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
  std::string calib_dir;
  std::string camera_1_dir;
  std::string camera_2_dir;
  std::ofstream gimbal_frame_file;
  std::ofstream gimbal_joint_file;
  std::ofstream gimbal_encoder_file;

  int nb_cameras = 0;
  std::string camera_1_topic;
  std::string camera_2_topic;

  int image_number = 0;
  cv::Mat image_1;
  cv::Mat image_2;
  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Quaternion gimbal_joint_body_orientation;

  CamCalibNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~CamCalibNode() {
    this->gimbal_frame_file.close();
    this->gimbal_joint_file.close();
    this->gimbal_encoder_file.close();
  }

  int configure(int hz);
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr &msg, cv::Mat &img);
  void image1Callback(const sensor_msgs::ImageConstPtr &msg);
  void image2Callback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointBodyCallback(const geometry_msgs::Quaternion &msg);
  void saveImages();
  void saveGimbalMeasurements();
  int loopCallback();
};

}  // namespace atl
#endif
