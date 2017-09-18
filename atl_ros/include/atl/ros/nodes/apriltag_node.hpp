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
static const std::string TARGET_W_POS_TOPIC = "/atl/apriltag/target/position/inertial";
static const std::string TARGET_W_YAW_TOPIC = "/atl/apriltag/target/yaw/inertial";
static const std::string TARGET_P_POS_TOPIC = "/atl/apriltag/target/position/body";
static const std::string TARGET_P_POS_ENCODER_TOPIC = "/atl/apriltag/target/position/body_encoders";
static const std::string TARGET_P_YAW_TOPIC = "/atl/apriltag/target/yaw/body";
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

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(const int hz);

  /**
   * Publish TagPose message
   *
   * @param tag Tag Pose
   */
  void publishTagPoseMsg(const TagPose &tag);

  /**
   * Publish target position in inertial frame
   *
   * @param gimbal_position Gimbal position in inertial frame
   * @param gimbal_orientation Gimbal orientation in inertial frame
   * @param target_B Detected target in body frame
   */
  void publishTargetInertialPositionMsg(const Vec3 &gimbal_position,
                                        const Quaternion &gimbal_orientation,
                                        const Vec3 &target_B);

  /**
   * Publish target yaw in inertial frame
   *
   * @param tag Tag pose in inertial frame
   * @param gimbal_frame Gimbal frame orientation in inertial frame
   */
  void publishTargetInertialYawMsg(const TagPose &tag,
                                   const Quaternion &gimbal_frame);

  /**
   * Publish target position in body frame
   *
   * @param target_P Target position in body planar frame
   */
  void publishTargetBodyPositionMsg(const Vec3 &target_P);

  /**
   * Publish target position in body frame (Encoder-version)
   *
   * @param target_P Target position in body planar frame
   */
  void publishTargetBodyPositionEncoderMsg(const Vec3 &target_P_encoder);

  /**
   * Publish target yaw in body frame
   *
   * @param tag Tag pose
   */
  void publishTargetBodyYawMsg(const TagPose &tag);

  /**
   * Image callback
   *
   * @param msg Image message
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};

} // namespace atl
#endif
