#include <sys/stat.h>

#include "atl/ros/nodes/cam_calib_node.hpp"


namespace atl {

int CamCalibNode::configure(const std::string &node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // camera
  ROS_GET_PARAM("/camera/config_dir", config_path);
  if (this->camera.configure(config_path) != 0) {
    ROS_ERROR("Failed to configure Camera!");
    return -2;
  };
  if (this->camera.initialize() != 0) {
    ROS_ERROR("Failed to initialize Camera!");
    return -3;
  }

  // calibration dir
  remove_dir("/tmp/calibration");
  int retval = mkdir("/tmp/calibration", ACCESSPERMS);
  if (retval != 0) {
    ROS_ERROR("Failed to create calibration dir!");
    return -4;
  }

  // register publisher and subscribers
  // clang-format off
  this->registerImagePublisher(CAMERA_IMAGE_TOPIC);
  this->registerSubscriber(GIMBAL_FRAME_ORIENTATION_TOPIC, &CamCalibNode::gimbalFrameCallback, this);
  this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointCallback, this);
  this->registerSubscriber(GIMBAL_ENCODER_ORIENTATION_TOPIC, &CamCalibNode::gimbalJointBodyCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&CamCalibNode::loopCallback, this));

  this->configured = true;
  return 0;
}

void CamCalibNode::gimbalFrameCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_frame_orientation);
}

void CamCalibNode::gimbalJointCallback(const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_orientation);
}

void CamCalibNode::gimbalJointBodyCallback(
  const geometry_msgs::Quaternion &msg) {
  convertMsg(msg, this->gimbal_joint_body_orientation);
}

int CamCalibNode::publishImage(cv::Mat &image) {
  sensor_msgs::ImageConstPtr img_msg;

  // clang-format off
  img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  this->img_pub.publish(img_msg);
  // clang-format on

  return 0;
}

int CamCalibNode::loopCallback() {
  double dist;
  cv::Mat image;
  int image_number = 0;

  // camera get and publish image
  this->camera.getFrame(image);
  this->publishImage(image);

  // show image
  cv::imshow("Image", image);
  int key = cv::waitKey(1);

  // parse keyboard input
  if (key == 32) {
    //     cv::imwrite(
    //       "/tmp/calibration/image_" + std::to_string(this->image_number) +
    //         ".jpg",
    //       this->image);
    image_number += 1;
  }

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::CamCalibNode, NODE_NAME, NODE_RATE);
