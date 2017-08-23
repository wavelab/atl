#include "atl/ros/nodes/camera_nodelet.hpp"
#include <pluginlib/class_list_macros.h>

namespace atl {

void CameraNodelet::onInit() {
  ROS_INFO("atl/camera_nodelet onInit");

  // Initialize all subscribers and publishers
  this->configureNodelet(NODE_RATE);

  // Get parameters
  std::string cam_config_path;
  this->nh.getParam("config_dir", cam_config_path);

  // camera
  if (this->camera.configure(cam_config_path) != 0) {
    ROS_ERROR("Failed to configure Camera!");
  }

  this->camera.initialize();

  // register publisher and subscribers
  // clang-format off
  this->registerImagePublisher(CAMERA_IMAGE_TOPIC);
  // this->registerSubscriber(GIMBAL_POSITION_TOPIC, &CameraNode::gimbalPositionCallback, this);
  // this->registerSubscriber(GIMBAL_FRAME_ORIENTATION_TOPIC, &CameraNode::gimbalFrameCallback, this);
  // this->registerSubscriber(GIMBAL_JOINT_ORIENTATION_TOPIC, &CameraNode::gimbalJointCallback, this);
  // this->registerSubscriber(APRILTAG_TOPIC, &CameraNode::aprilTagCallback, this);
  // clang-format on
  this->registerShutdown(SHUTDOWN_TOPIC);

  // register loop callback
  this->registerLoopCallback(std::bind(&CameraNodelet::loopCallback, this));

  this->configured = true;
}

int CameraNodelet::publishImage() {
  sensor_msgs::ImageConstPtr img_msg;

  // encode position and orientation into image (first 11 pixels in first row)
  // if (this->gimbal_mode) {
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
  // }

  // clang-format off
  img_msg = cv_bridge::CvImage(
    std_msgs::Header(),
    "bgr8",
    this->image
  ).toImageMsg();
  this->img_pubs[CAMERA_IMAGE_TOPIC].publish(img_msg);
  // clang-format on

  return 0;
}

void CameraNodelet::gimbalPositionCallback(
    const geometry_msgs::Vector3ConstPtr &msg) {
  convertMsg(msg, this->gimbal_position);
}

void CameraNodelet::gimbalFrameCallback(
    const geometry_msgs::QuaternionConstPtr &msg) {
  convertMsg(msg, this->gimbal_frame_orientation);
}

void CameraNodelet::gimbalJointCallback(
    const geometry_msgs::QuaternionConstPtr &msg) {
  convertMsg(msg, this->gimbal_joint_orientation);
}

void CameraNodelet::aprilTagCallback(
    const atl_msgs::AprilTagPoseConstPtr &msg) {
  convertMsg(msg, this->tag);
}

int CameraNodelet::loopCallback() {
  double dist;

  // change mode depending on apriltag distance
  if (!this->tag.detected) {
    this->camera.changeMode("640x640");
  } else {
    dist = this->tag.position(2);
    if (dist > 8.0) {
      this->camera.changeMode("640x640");
    } else if (dist > 4.0) {
      this->camera.changeMode("320x320");
    } else {
      this->camera.changeMode("160x160");
    }
  }

  this->camera.showImage(this->image);
  this->camera.getFrame(this->image);
  this->publishImage();

  return 0;
}
} // namespace atl

PLUGINLIB_DECLARE_CLASS(atl,
                        CameraNodelet,
                        atl::CameraNodelet,
                        atl::ROSNodelet);
