#ifndef WAVESIM_ROS_NODES_CAMERA_NODE_HPP
#define WAVESIM_ROS_NODES_CAMERA_NODE_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>

#include "wave/utils/math.hpp"
#include "wavesim_ros/utils/node.hpp"
#include "wavesim_gazebo/clients/camera_gclient.hpp"

namespace wavesim {
namespace ros {

using namespace wave;

// NODE SETTINGS
#define NODE_NAME "wavesim_camera"
#define NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMAGE_RTOPIC "/wavesim/camera/image"

// SUBSCRIBE TOPICS
#define CAMERA_MODE_RTOPIC "/wavesim/camera/mode"

class CameraNode : public gaz::CameraGClient, public ROSNode {
public:
  bool configured;

  bool gimbal_mode;
  std::string gimbal_position_topic;
  std::string gimbal_frame_orientation_topic;
  std::string gimbal_joint_orientation_topic;
  Vec3 gimbal_position;
  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;

  std::string camera_mode;

  CameraNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->configured = false;

    this->gimbal_mode = false;
    this->gimbal_position_topic = "";
    this->gimbal_frame_orientation_topic = "";
    this->gimbal_joint_orientation_topic = "";
    this->gimbal_position = Vec3();
    this->gimbal_frame_orientation = Quaternion();
    this->gimbal_joint_orientation = Quaternion();

    this->camera_mode = "640x640";
  }

  int configure(const std::string &node_name, int hz);
  void gimbalPositionCallback(const geometry_msgs::Vector3 &msg);
  void gimbalFrameOrientationCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointOrientationCallback(const geometry_msgs::Quaternion &msg);
  void modeCallback(const std_msgs::String &msg);
  void imageCallback(ConstImagePtr &msg);
};

}  // end of ros namespace
}  // end of wavesim namespace
#endif
