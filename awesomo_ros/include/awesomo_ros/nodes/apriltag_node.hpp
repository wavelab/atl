#ifndef __AWESOMO_ROS_NODES_APRILTAG_NODE_HPP__
#define __AWESOMO_ROS_NODES_APRILTAG_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

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
#define TARGET_IF_TOPIC "/awesomo/apriltag/target/inertial"
#define TARGET_BPF_TOPIC "/awesomo/apriltag/target/body"

// SUBSCRIBE TOPICS
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image_pose_stamped"
#define SHUTDOWN "/awesomo/apriltag/shutdown"

class AprilTagNode : public ROSNode {
public:
  MITDetector detector;
  Pose camera_offset;

  AprilTagNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void publishTagPoseMsg(TagPose tag);
  void publishTargetBodyPositionMsg(Vec3 target_bf);
  void publishTargetInertialPositionMsg(Vec3 gimbal_position, Vec3 target_bf);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  Vec3 getTargetInBF(Vec3 target_cf);
  Vec3 getTargetInBPF(Vec3 target_cf, Quaternion joint_if);
};

}  // end of awesomo namespace
#endif
