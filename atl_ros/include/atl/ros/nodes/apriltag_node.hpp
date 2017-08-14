#ifndef ATL_ROS_NODES_APRILTAG_NODE_HPP
#define ATL_ROS_NODES_APRILTAG_NODE_HPP

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <atl/atl_core.hpp>
#include <atl_msgs/AprilTagPose.h>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// NODE SETTINGS
static const double NODE_RATE = 100;

// PUBLISH TOPICS
// clang-format off
static const std::string TARGET_POSE_TOPIC = "/atl/apriltag/target";
static const std::string TARGET_IF_POS_TOPIC = "/atl/apriltag/target/position/inertial";
static const std::string TARGET_IF_YAW_TOPIC = "/atl/apriltag/target/yaw/inertial";
static const std::string TARGET_BPF_POS_TOPIC = "/atl/apriltag/target/position/body";
static const std::string TARGET_BPF_POS_ENCODER_TOPIC = "/atl/apriltag/target/position/body_encoders";
static const std::string TARGET_BPF_YAW_TOPIC = "/atl/apriltag/target/yaw/body";
// clang-format on

// SUBSCRIBE TOPICS
static const std::string CAMERA_IMAGE_TOPIC = "/atl/camera/image";
static const std::string SHUTDOWN = "/atl/apriltag/shutdown";

namespace atl {

class AprilTagNode : public ROSNode {
public:
  MITDetector detector;
  Pose camera_offset;

  AprilTagNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(int hz);
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

} // namespace atl
#endif
