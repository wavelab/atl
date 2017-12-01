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
  std::ofstream gimbal_imu_file;
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
  Quaternion gimbal_imu;
  Quaternion gimbal_encoder;

  CamCalibNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~CamCalibNode() {
    this->gimbal_imu_file.close();
    this->gimbal_encoder_file.close();
  }

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(const int hz);

  /**
   * Convert image message to cv::Mat
   * @param msg Image message
   * @param img OpenCV cv::Mat
   */
  void imageMsgToCvMat(const sensor_msgs::ImageConstPtr &msg, cv::Mat &img);

  /**
   * Static camera callback
   * @param msg Image message
   */
  void staticCameraCallback(const sensor_msgs::ImageConstPtr &msg);

  /**
   * Gimbal camera callback
   * @param msg Image message
   */
  void gimbalCameraCallback(const sensor_msgs::ImageConstPtr &msg);

  /**
   * Gimbal joint callback
   * @param msg Gimbal joint orientation message
   */
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);

  /**
   * Gimbal joint encoder callback
   * @param msg Gimbal joint orientation message
   */
  void gimbalJointEncoderCallback(const geometry_msgs::Quaternion &msg);

  /**
   * Check if chessboard is detected
   * @return Boolean to denote success or failure
   */
  bool chessboardDetected();

  /**
   * Show images
   */
  void showImages();

  /**
   * Save images
   */
  void saveImages();

  /**
   * Save gimbal measurements
   */
  void saveGimbalMeasurements();

  /**
   * Loop callback
   */
  int loopCallback();
};

} // namespace atl
#endif
