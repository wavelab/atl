#ifndef __AWESOMO_ROS_NODES_PG_CAMERA_NODE_HPP__
#define __AWESOMO_ROS_NODES_PG_CAMERA_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_camera"
#define NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image"

// SUBSCRIBE TOPICS
#define APRILTAG_TOPIC "/awesomo/apriltag/target"
#define GIMBAL_FRAME_ORIENTATION_TOPIC "/awesomo/gimbal/frame/orientation/inertial"
#define GIMBAL_JOINT_ORIENTATION_TOPIC "/awesomo/gimbal/joint/orientation/inertial"
#define LT_BODY_POSITION_TOPIC "/awesomo/estimate/landing_target/position/body"
#define LT_DETECTED_TOPIC "/awesomo/estimate/landing_target/detected"
#define SHUTDOWN_TOPIC "/awesomo/camera/shutdown"

class PGCameraNode : public ROSNode {
public:
  PointGreyCamera camera;
  cv::Mat image;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Vec3 gimbal_position;
  bool target_detected;
  Vec3 target_pos_bf;

  PGCameraNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->gimbal_frame_orientation = Quaternion();
    this->gimbal_joint_orientation = Quaternion();
    this->gimbal_position = Vec3();
    this->target_detected = false;
    this->target_pos_bf = Vec3();
  }
  int configure(std::string node_name, int hz);
  int publishImage(void);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void targetPositionCallback(const geometry_msgs::Vector3 &msg);
  void targetDetectedCallback(const std_msgs::Bool &msg);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
