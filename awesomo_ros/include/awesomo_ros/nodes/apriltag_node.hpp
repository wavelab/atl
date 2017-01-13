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
#define TARGET_IF_TOPIC "/awesomo/apriltag/target/inertial_frame"
#define TARGET_BPF_TOPIC "/awesomo/apriltag/target/body_planar_frame"
#define GIMBAL_SETPOINT_ATTITUDE_TOPIC "/awesomo/gimbal/setpoint/attitude"

// SUBSCRIBE TOPICS
#define GIMBAL_FRAME_POSE_TOPIC "/awesomo/gimbal/frame/pose"
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image_imu"
#define SHUTDOWN "/awesomo/apriltag/shutdown"

class AprilTagNode : public ROSNode {
public:
  MITDetector detector;
  Pose camera_offset;

  Vec3 gimbal_position;
  Quaternion gimbal_joint_orientation;
  Vec3 target_bpf;

  AprilTagNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void publishTagPoseMsg(TagPose tag);
  void publishTargetBodyPositionMsg(void);
  void publishTargetInertialPositionMsg(void);
  void publishGimbalSetpointAttitudeMsg(Vec3 setpoints);
  void gimbalFrameCallback(const geometry_msgs::Pose &msg);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  Vec3 getTargetPositionInBodyFrame(Vec3 target_cf);
  Vec3 getTargetPositionInBodyPlanarFrame(Vec3 target_cf);
  void trackTarget(void);
};

}  // end of awesomo namespace
#endif
