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
#define TARGET_IF_YAW_TOPIC "/awesomo/apriltag/target/yaw/inertial"
#define TARGET_BPF_POS_TOPIC "/awesomo/apriltag/target/position/body"
#define TARGET_BPF_POS_ENCODER_TOPIC "/awesomo/apriltag/target/position/body_encoders"
#define TARGET_BPF_YAW_TOPIC "/awesomo/apriltag/target/yaw/body"

#define TARGET_APR_GROUND_TRUTH "/awesomo/apriltag/target/ground_truth"
#define TARGET_CAM_GROUND_TRUTH "/awesomo/apriltag/target/camera_ground_truth"
#define TARGET_BPF_GROUND_TRUTH "/awesomo/apriltag/target/camera_ground_truth"

// SUBSCRIBE TOPICS
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image"
#define CAMERA_MOCAP_POSE_TOPIC "/mocap/camera/pose"
#define SHUTDOWN "/awesomo/apriltag/shutdown"

class AprilTagNode : public ROSNode {
public:
  // MITDetector detector;
  MichiganDetector detector;
  Pose camera_offset;

  Vec3 gimbal_position;
  Vec3 target_bpf_encoder;
  Vec3 quad_position;

  Quaternion gimbal_frame;
  Quaternion gimbal_joint;
  Quaternion quad_orientation;
  Quaternion gimbal_joint_bf;

  bool use_mocap;

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
  void mocapCallback(const geometry_msgs::PoseStamped &msg);
  int calcTargetGroundTruth(Vec3 &ground_truth, Vec3 inertial_position);
  int calcCameraGroundTruth(Vec3 &apr_ground_truth);
  int calcBPFGroundTruth(Vec3 &cam_ground_truth);
  int rotateMocapGimbalPosition();
};

}  // end of awesomo namespace
#endif
