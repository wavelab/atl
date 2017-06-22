#ifndef __atl_ROS_NODES_PG_CAMERA_NODE_HPP__
#define __atl_ROS_NODES_PG_CAMERA_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <dji_sdk/dji_drone.h>
#include <atl_core/atl_core.hpp>

#include "atl_ros/utils/node.hpp"
#include "atl_ros/utils/msgs.hpp"


namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_camera"
#define NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMAGE_TOPIC "/atl/camera/image"

// SUBSCRIBE TOPICS
#define APRILTAG_TOPIC "/atl/apriltag/target"
#define GIMBAL_POSITION_TOPIC "/atl/gimbal/position/inertial"
#define GIMBAL_FRAME_ORIENTATION_TOPIC "/atl/gimbal/frame/orientation/inertial"
#define GIMBAL_JOINT_ORIENTATION_TOPIC "/atl/gimbal/joint/orientation/inertial"
#define ENCODER_ORIENTATION_TOPIC "atl/gimbal/joint/orientation/body"
#define LT_BODY_POSITION_TOPIC "/atl/estimate/landing_target/position/body"

#define QUAD_POSITION_TOPIC "/dji_sdk/local_position"
#define QUAD_ORIENTATION_TOPIC "/dji_sdk/attitude_quaternion"

#define LT_DETECTED_TOPIC "/atl/estimate/landing_target/detected"
#define SHUTDOWN_TOPIC "/atl/camera/shutdown"


class PGCameraNode : public ROSNode {
public:
  PointGreyCamera camera;
  cv::Mat image;

  Vec3 gimbal_position;
  Vec3 quadrotor_position;
  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Quaternion gimbal_joint_body_orientation;
  Quaternion quadrotor_orientation;

  bool target_detected;
  Vec3 target_pos_bf;
  bool photobooth = false;
  int image_number = 0;

  PGCameraNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->gimbal_position = Vec3();
    this->gimbal_frame_orientation = Quaternion();
    this->gimbal_joint_orientation = Quaternion();

    this->target_detected = false;
    this->target_pos_bf = Vec3();
  }

  int configure(std::string node_name, int hz);
  int publishImage(void);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalPositionCallback(const geometry_msgs::Vector3 &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointBodyCallback(const geometry_msgs::Quaternion &msg);
  void targetPositionCallback(const geometry_msgs::Vector3 &msg);
  void targetDetectedCallback(const std_msgs::Bool &msg);

  void quadPositionCallback(const dji_sdk::LocalPosition &msg);
  void quadOrientationCallback(const dji_sdk::AttitudeQuaternion &msg);

  int loopCallback(void);
};

}  // end of atl namespace
#endif
