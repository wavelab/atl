#ifndef __atl_ROS_NODES_APRILTAG_NODE_HPP__
#define __atl_ROS_NODES_APRILTAG_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

#include <atl_core/atl_core.hpp>
#include <atl_msgs/AprilTagPose.h>

#include "atl_ros/utils/node.hpp"
#include "atl_ros/utils/msgs.hpp"


namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_apriltag"
#define NODE_RATE 100

// PUBLISH TOPICS
#define TARGET_POSE_TOPIC "/atl/apriltag/target"
#define TARGET_IF_POS_TOPIC "/atl/apriltag/target/position/inertial"
#define TARGET_IF_YAW_TOPIC "/atl/apriltag/target/yaw/inertial"
#define TARGET_BPF_POS_TOPIC "/atl/apriltag/target/position/body"
#define TARGET_BPF_POS_ENCODER_TOPIC "/atl/apriltag/target/position/body_encoders"
#define TARGET_BPF_YAW_TOPIC "/atl/apriltag/target/yaw/body"

// SUBSCRIBE TOPICS
#define CAMERA_IMAGE_TOPIC "/atl/camera/image"
#define SHUTDOWN "/atl/apriltag/shutdown"

class AprilTagNode : public ROSNode {
public:
  MITDetector detector;
  Pose camera_offset;

  AprilTagNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void publishTagPoseMsg(TagPose tag);
  void publishTargetInertialPositionMsg(Vec3 gimbal_position,
                                        Quaternion gimbal_orientation,
                                        Vec3 target_bf);
  void publishTargetInertialYawMsg(TagPose tag, Quaternion gimbal_frame);
  void publishTargetBodyPositionMsg(Vec3 target_bpf);
  void publishTargetBodyPositionEncoderMsg(Vec3 target_bpf_encoder);
  void publishTargetBodyYawMsg(TagPose tag);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};

}  // end of atl namespace
#endif
