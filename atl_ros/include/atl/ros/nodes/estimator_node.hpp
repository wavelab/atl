#ifndef ATL_ROS_NODES_ESTIMATOR_NODE_HPP
#define ATL_ROS_NODES_ESTIMATOR_NODE_HPP

#include <ros/ros.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/node.hpp"
#include "atl/ros/utils/msgs.hpp"

namespace atl {

#define KF_MODE 1
#define EKF_MODE 2

#define ESTIMATOR_ON 1
#define ESTIMATOR_OFF -1

// NODE SETTINGS
#define NODE_NAME "atl_estimator"
#define NODE_RATE 50

// PUBLISH TOPICS
#define LT_BODY_POSITION_TOPIC "/atl/estimate/landing_target/position/body"
#define LT_BODY_VELOCITY_TOPIC "/atl/estimate/landing_target/velocity/body"
#define LT_DETECTED_TOPIC "/atl/estimate/landing_target/detected"
#define GIMBAL_SETPOINT_ATTITUDE_TOPIC "/atl/gimbal/setpoint/attitude"
#define QUAD_YAW_TOPIC "/atl/control/yaw/set"

// SUBSCRIBE TOPICS
#define QUAD_POSE_TOPIC "/atl/quadrotor/pose/local"
#define QUAD_VELOCITY_TOPIC "/atl/quadrotor/velocity/local"
#define ESTIMATOR_ON_TOPIC "/atl/estimator/on"
#define ESTIMATOR_OFF_TOPIC "/atl/estimator/off"
#define TARGET_BF_POS_TOPIC "/atl/apriltag/target/position/body"
#define TARGET_IF_YAW_TOPIC "/atl/apriltag/target/yaw/inertial"

class EstimatorNode : public ROSNode {
public:
  int mode;
  int state;
  bool initialized;

  std::string quad_frame;

  KalmanFilterTracker kf_tracker;
  ExtendedKalmanFilterTracker ekf_tracker;

  Pose quad_pose;
  Vec3 quad_velocity;

  std::vector<double> target_pos_x_init;
  std::vector<double> target_pos_y_init;
  std::vector<double> target_pos_z_init;

  bool target_detected;
  bool target_losted;
  Vec3 target_pos_bpf;
  Vec3 target_vel_bpf;
  double target_yaw_wf;
  Vec3 target_measured;
  Vec3 target_last_measured;

  struct timespec target_last_updated;
  double target_lost_threshold;

  EstimatorNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->quad_frame = "";

    this->kf_tracker = KalmanFilterTracker();
    this->ekf_tracker = ExtendedKalmanFilterTracker();

    this->quad_pose = Pose();
    this->target_detected = false;
    this->target_losted = true;
    this->target_pos_bpf << 0.0, 0.0, 0.0;
    this->target_vel_bpf << 0.0, 0.0, 0.0;
    this->target_yaw_wf = 0.0;
    this->target_measured << 0.0, 0.0, 0.0;
    this->target_last_measured << 0.0, 0.0, 0.0;
    this->target_last_updated = (struct timespec){0};
    this->target_lost_threshold = 1000.0;
  }

  int configure(std::string node_name, int hz);
  void initLTKF(Vec3 target_measured);
  void resetLTKF(Vec3 target_measured);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void quadVelocityCallback(const geometry_msgs::TwistStamped &msg);
  void onCallback(const std_msgs::Bool &msg);
  void offCallback(const std_msgs::Bool &msg);
  void targetBodyPosCallback(const geometry_msgs::Vector3 &msg);
  void targetInertialPosCallback(const geometry_msgs::Vector3 &msg);
  void targetInertialYawCallback(const std_msgs::Float64 &msg);
  void publishLTKFBodyPositionEstimate(void);
  void publishLTKFBodyVelocityEstimate(void);
  void publishLTDetected(void);
  void publishGimbalSetpointAttitudeMsg(Vec3 setpoints);
  void publishQuadYawMsg(void);
  void trackTarget(void);
  void reset(void);
  int estimateKF(double dt);
  int estimateEKF(double dt);
  int estimate(void);
  int loopCallback(void);
};

}  // namespace atl
#endif
