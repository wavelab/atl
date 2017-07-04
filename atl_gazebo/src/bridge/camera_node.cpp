#include "atl_gazebo/bridge/camera_node.hpp"

namespace atl {
namespace ros {

int CameraNode::configure(const std::string &node_name, int hz) {
  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // configure ros node
  // clang-format off
  this->ros_nh->getParam("/gimbal_mode", this->gimbal_mode);
  this->ros_nh->getParam("/gimbal_position_topic", this->gimbal_position_topic);
  this->ros_nh->getParam("/gimbal_frame_orientation_topic", this->gimbal_frame_orientation_topic);
  this->ros_nh->getParam("/gimbal_joint_orientation_topic", this->gimbal_joint_orientation_topic);
  // clang-format on

  if (this->gimbal_mode) {
    // clang-format off
    ROSNode::registerImagePublisher(CAMERA_IMAGE_RTOPIC);
    ROSNode::registerSubscriber(this->gimbal_position_topic, &CameraNode::gimbalPositionCallback, this);
    ROSNode::registerSubscriber(this->gimbal_frame_orientation_topic, &CameraNode::gimbalFrameOrientationCallback, this);
    ROSNode::registerSubscriber(this->gimbal_joint_orientation_topic, &CameraNode::gimbalJointOrientationCallback, this);
    ROSNode::registerSubscriber(CAMERA_MODE_RTOPIC, &CameraNode::modeCallback, this);
    // clang-format on
  } else {
    ROSNode::registerImagePublisher(CAMERA_IMAGE_RTOPIC);
  }

  // setup gazebo client
  if (CameraGClient::configure() != 0) {
    ROS_ERROR("Failed to configure CameraGClient!");
    return -1;
  }

  this->configured = true;
  return 0;
}

void CameraNode::gimbalPositionCallback(const geometry_msgs::Vector3 &msg) {
  this->gimbal_position(0) = msg.x;
  this->gimbal_position(1) = msg.y;
  this->gimbal_position(2) = msg.z;
}

void CameraNode::gimbalFrameOrientationCallback(
  const geometry_msgs::Quaternion &msg) {
  this->gimbal_frame_orientation.w() = msg.w;
  this->gimbal_frame_orientation.x() = msg.x;
  this->gimbal_frame_orientation.y() = msg.y;
  this->gimbal_frame_orientation.z() = msg.z;
}

void CameraNode::gimbalJointOrientationCallback(
  const geometry_msgs::Quaternion &msg) {
  this->gimbal_joint_orientation.w() = msg.w;
  this->gimbal_joint_orientation.x() = msg.x;
  this->gimbal_joint_orientation.y() = msg.y;
  this->gimbal_joint_orientation.z() = msg.z;
}

void CameraNode::modeCallback(const std_msgs::String &msg) {
  ROS_INFO("Camera Mode: [%s]", msg.data.c_str());
  this->camera_mode = msg.data;
}

void CameraNode::imageCallback(ConstImagePtr &msg) {
  CameraGClient::imageCallback(msg);

  // change mode
  cv::Size image_size;
  if (this->camera_mode == "640x640") {
    image_size = cv::Size(640, 640);
  } else if (this->camera_mode == "320x320") {
    image_size = cv::Size(320, 320);
  } else if (this->camera_mode == "160x160") {
    image_size = cv::Size(160, 160);
  }
  cv::resize(this->image, this->image, image_size);

  // encode position and orientation into image (first 11 pixels in first row)
  if (this->gimbal_mode) {
    this->image.at<double>(0, 0) = this->gimbal_position(0);
    this->image.at<double>(0, 1) = this->gimbal_position(1);
    this->image.at<double>(0, 2) = this->gimbal_position(2);

    this->image.at<double>(0, 3) = this->gimbal_frame_orientation.w();
    this->image.at<double>(0, 4) = this->gimbal_frame_orientation.x();
    this->image.at<double>(0, 5) = this->gimbal_frame_orientation.y();
    this->image.at<double>(0, 6) = this->gimbal_frame_orientation.z();

    this->image.at<double>(0, 7) = this->gimbal_joint_orientation.w();
    this->image.at<double>(0, 8) = this->gimbal_joint_orientation.x();
    this->image.at<double>(0, 9) = this->gimbal_joint_orientation.y();
    this->image.at<double>(0, 10) = this->gimbal_joint_orientation.z();
  }

  // build image msg
  // clang-format off
  sensor_msgs::ImageConstPtr img_msg;
  img_msg = cv_bridge::CvImage(
    std_msgs::Header(),
    "bgr8",
    this->image
  ).toImageMsg();
  // clang-format on

  // publish image
  this->img_pub.publish(img_msg);

  // debug
  if (this->debug_mode) {
    cv::imshow("CameraNode Image", this->image);
    cv::waitKey(1);
  }
}

}  // namespace ros
}  // namespace atl

RUN_ROS_NODE(atl::ros::CameraNode, NODE_NAME, NODE_RATE);
