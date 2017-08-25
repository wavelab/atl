#ifndef ATL_ROS_NODES_CAMERA_NODELET_HPP
#define ATL_ROS_NODES_CAMERA_NODELET_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>

#include "atl/atl_core.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/ros_topic_manager.hpp"

// NODE SETTINGS
static const int NODE_RATE = 100;

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

class CameraNodelet : public nodelet::Nodelet {
public:
  CameraNodelet() {}

  Camera camera;
  cv::Mat image;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Vec3 gimbal_position;
  TagPose tag;

  int publishImage();
  void gimbalPositionCallback(const geometry_msgs::Vector3ConstPtr &msg);
  void gimbalFrameCallback(const geometry_msgs::QuaternionConstPtr &msg);
  void gimbalJointCallback(const geometry_msgs::QuaternionConstPtr &msg);
  void aprilTagCallback(const atl_msgs::AprilTagPoseConstPtr &msg);
  int timerCallback();

private:
  virtual void onInit();

  ROSTopicManager ros_topic_manager;
  ros::Timer timer;
};
} // namespace atl

#endif // ATL_ROS_NODES_CAMERA_NODELET_HPP
