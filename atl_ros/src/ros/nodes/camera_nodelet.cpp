#include "atl/ros/nodes/camera_nodelet.hpp"
#include <pluginlib/class_list_macros.h>

namespace atl {

void CameraNodelet::onInit() {
  ROS_INFO("atl/atl_camera_nodelet onInit");
  // Create node handle
  this->nh = getNodeHandle();

  // Get parameters
  std::string cam_config_path;

  if (this->nh.getParam("config_dir", cam_config_path)) {
    ROS_INFO("Loaded camera config directory.");
  } else {
    ROS_ERROR("Could not load camera config directory.");
    ros::shutdown();
  }

  // camera
  if (this->camera.configure(cam_config_path) != 0) {
    ROS_ERROR("Failed to configure Camera!");
    ros::shutdown();
  }

  this->camera.initialize();

  // register publisher and subscribers
  // clang-format off
  this->ros_topic_manager.registerImagePublisher(this->nh, CAMERA_IMAGE_TOPIC);
  // this->ros_topic_manager.registerSubscriber(nh, GIMBAL_POSITION_TOPIC, &CameraNodelet::gimbalPositionCallback, this);
  // this->ros_topic_manager.registerSubscriber(nh, GIMBAL_FRAME_ORIENTATION_TOPIC, &CameraNodelet::gimbalFrameCallback, this);
  // this->ros_topic_manager.registerSubscriber(nh, GIMBAL_JOINT_ORIENTATION_TOPIC, &CameraNodelet::gimbalJointCallback, this);
  // this->ros_topic_manager.registerSubscriber(nh, APRILTAG_TOPIC, &CameraNodelet::aprilTagCallback, this);
  // clang-format on
  this->ros_topic_manager.registerShutdown(this->nh, SHUTDOWN_TOPIC);

  double period = 1.0 / NODE_RATE;
  this->timer =
      this->nh.createTimer(ros::Duration(period),
                           boost::bind(&CameraNodelet::timerCallback, this));
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
  this->ros_topic_manager.img_pubs[CAMERA_IMAGE_TOPIC].publish(img_msg);
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

int CameraNodelet::timerCallback() {
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

PLUGINLIB_DECLARE_CLASS(atl_ros,
                        CameraNodelet,
                        atl::CameraNodelet,
                        nodelet::Nodelet);
