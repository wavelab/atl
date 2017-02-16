#ifndef __AWESOMO_ROS_NODES_XIMEA_CAMERA_NODE_HPP__
#define __AWESOMO_ROS_NODES_XIMEA_CAMERA_NODE_HPP__

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
#define APRILTAG_TOPIC "/awesomo/apriltag/target"
#define SHUTDOWN_TOPIC "/awesomo/camera/shutdown"

class XimeaCameraNode : public ROSNode {
public:
  XimeaCamera camera;
  cv::Mat image;

  Quaternion gimbal_frame_orientation;
  Quaternion gimbal_joint_orientation;
  Vec3 gimbal_position;
  TagPose tag;

  XimeaCameraNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  int publishImage(void);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void gimbalFrameCallback(const geometry_msgs::Quaternion &msg);
  void gimbalJointCallback(const geometry_msgs::Quaternion &msg);
  void aprilTagCallback(const awesomo_msgs::AprilTagPose &msg);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
