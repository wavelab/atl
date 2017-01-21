#ifndef __AWESOMO_ROS_NODES_APRILTAG_NODE_HPP__
#define __AWESOMO_ROS_NODES_APRILTAG_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include <awesomo_core/awesomo_core.hpp>
#include <awesomo_msgs/AprilTagPose.h>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_apriltag"
#define NODE_RATE 100

// PUBLISH TOPICS
#define TARGET_POSE_TOPIC "/awesomo/apriltag/target"
#define TARGET_IF_POS_TOPIC "/awesomo/apriltag/target/position/inertial"
#define TARGET_BPF_POS_TOPIC "/awesomo/apriltag/target/position/body"
#define TARGET_IF_YAW_TOPIC "/awesomo/apriltag/target/yaw/inertial"
#define TARGET_BPF_YAW_TOPIC "/awesomo/apriltag/target/yaw/body"

// SUBSCRIBE TOPICS
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image_pose_stamped"
#define MAVROS_POSE_TOPIC "/mavros/local_position/pose"
#define SHUTDOWN "/awesomo/apriltag/shutdown"

class AprilTagNode : public ROSNode {
public:
  MITDetector detector;
  Pose camera_offset;
  Quaternion orientation;

  AprilTagNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void publishTagPoseMsg(TagPose tag);
  void publishTargetInertialPositionMsg(Vec3 gimbal_position, Vec3 target_bf);
  void publishTargetInertialYawMsg(TagPose tag, Quaternion body);
  void publishTargetBodyPositionMsg(Vec3 target_bf);
  void publishTargetBodyYawMsg(TagPose tag);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  void poseCallback(const geometry_msgs::PoseStamped &msg);
};

}  // end of awesomo namespace
#endif
