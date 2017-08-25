#ifndef ATL_ROS_NODES_ESTIMATOR_NODE_HPP
#define ATL_ROS_NODES_ESTIMATOR_NODE_HPP

#include <ros/ros.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

#define ESTIMATOR_ON 1
#define ESTIMATOR_OFF -1

// NODE SETTINGS
static const double NODE_RATE = 50;

// clang-format off
// PUBLISH TOPICS
static const std::string LT_BODY_POSITION_TOPIC = "/atl/estimate/landing_target/position/body";
static const std::string LT_BODY_VELOCITY_TOPIC = "/atl/estimate/landing_target/velocity/body";
static const std::string LT_DETECTED_TOPIC = "/atl/estimate/landing_target/detected";
static const std::string GIMBAL_SETPOINT_ATTITUDE_TOPIC = "/atl/gimbal/setpoint/attitude";
static const std::string QUAD_YAW_TOPIC = "/atl/control/yaw/set";

// SUBSCRIBE TOPICS
static const std::string QUAD_POSE_TOPIC = "/atl/quadrotor/pose/local";
static const std::string QUAD_VELOCITY_TOPIC = "/atl/quadrotor/velocity/local";
static const std::string ESTIMATOR_ON_TOPIC = "/atl/estimator/on";
static const std::string ESTIMATOR_OFF_TOPIC = "/atl/estimator/off";
static const std::string TARGET_BF_POS_TOPIC = "/atl/apriltag/target/position/body";
static const std::string TARGET_IF_YAW_TOPIC = "/atl/apriltag/target/yaw/inertial";
// clang-format on

namespace atl {

class EstimatorNode : public ROSNode {
public:
  std::string estimator_type;
  bool running = false;
  bool initialized = false;

  KFTracker kf_tracker;
  EKFTracker ekf_tracker;

  Pose quad_pose;
  Vec3 quad_velocity{0.0, 0.0, 0.0};

  std::vector<double> target_pos_x_init;
  std::vector<double> target_pos_y_init;
  std::vector<double> target_pos_z_init;

  bool target_detected = false;
  bool target_losted = true;
  Vec3 target_pos_bpf{0.0, 0.0, 0.0};
  Vec3 target_vel_bpf{0.0, 0.0, 0.0};
  double target_yaw_wf = 0.0;
  Vec3 target_measured{0.0, 0.0, 0.0};
  Vec3 target_last_measured{0.0, 0.0, 0.0};

  struct timespec target_last_updated = (struct timespec){0};
  double target_lost_threshold = 1000.0;

  EstimatorNode(int argc, char **argv) : ROSNode(argc, argv) {}

  int configure(const int hz);
  void initLTKF(const Vec3 &target_measured);
  void resetLTKF(const Vec3 &target_measured);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void quadVelocityCallback(const geometry_msgs::TwistStamped &msg);
  void onCallback(const std_msgs::Bool &msg);
  void offCallback(const std_msgs::Bool &msg);

  /**
   * Landing target position (body frame) callback
   */
  void targetBodyPosCallback(const geometry_msgs::Vector3 &msg);

  /**
   * Landing target position (inertial frame) callback
   */
  void targetInertialPosCallback(const geometry_msgs::Vector3 &msg);

  /**
   * Landing target yaw (inertial frame) callback
   */
  void targetInertialYawCallback(const std_msgs::Float64 &msg);

  /**
   * Publish landing target position relative to quadrotor (body frame)
   */
  void publishLTKFBodyPositionEstimate();

  /**
   * Publish landing target velocity relative to quadrotor (body frame)
   */
  void publishLTKFBodyVelocityEstimate();

  /**
   * Publish detected landing target
   */
  void publishLTDetected();

  /**
   * Publish gimbal attitude setpoints
   */
  void publishGimbalSetpointAttitudeMsg(Vec3 setpoints);

  /**
   * Publish quadrotor yaw in inertial frame
   */
  void publishQuadYawMsg();

  /**
   * Track target
   */
  void trackTarget();

  /**
   * Reset estimator
   */
  void reset();

  /**
   * Estimate with Kalman Filter
   */
  int estimateKF(const double dt);

  /**
   * Estimate with Extended Kalman Filter
   */
  int estimateEKF(const double dt);

  /**
   * Estimate
   */
  int estimate();

  /**
   * Loop callback
   */
  int loopCallback();
};

} // namespace atl
#endif
