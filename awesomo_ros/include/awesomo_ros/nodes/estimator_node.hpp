#ifndef __AWESOMO_ROS_NODES_ESTIMATOR_NODE_HPP__
#define __AWESOMO_ROS_NODES_ESTIMATOR_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"

namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_estimator"
#define NODE_RATE 200

// PUBLISH TOPICS
#define LT_INERTIAL_POSITION_TOPIC "/awesomo/estimate/landing_target/position/inertial"
#define LT_INERTIAL_VELOCITY_TOPIC "/awesomo/estimate/landing_target/velocity/inertial"
#define LT_BODY_POSITION_TOPIC "/awesomo/estimate/landing_target/position/body"
#define LT_BODY_VELOCITY_TOPIC "/awesomo/estimate/landing_target/velocity/body"
#define LT_DETECTED_TOPIC "/awesomo/estimate/landing_target/detected"
#define GIMBAL_SETPOINT_ATTITUDE_TOPIC "/awesomo/gimbal/setpoint/attitude"

// SUBSCRIBE TOPICS
#define QUAD_POSE_TOPIC "/mavros/local_position/pose"
#define QUAD_VELOCITY_TOPIC "/mavros/local_position/velocity"
#define TARGET_INERTIAL_TOPIC "/awesomo/apriltag/target/position/inertial"

class EstimatorNode : public ROSNode {
public:
  KalmanFilterTracker estimator;

  Pose quad_pose;
  Vec3 quad_velocity;
  bool target_detected;
  Vec3 target_pos_bpf;
  Vec3 target_vel_bpf;
  Vec3 target_pos_wf;
  Vec3 target_last_pos_wf;
  struct timespec target_last_updated;
  double target_lost_threshold;

  EstimatorNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->estimator = KalmanFilterTracker();

    this->quad_pose = Pose();
    this->target_detected = false;
    this->target_pos_bpf << 0.0, 0.0, 0.0;
    this->target_vel_bpf << 0.0, 0.0, 0.0;
    this->target_pos_wf << 0.0, 0.0, 0.0;
    this->target_last_pos_wf << 0.0, 0.0, 0.0;
    this->target_last_updated = (struct timespec){0};
    this->target_lost_threshold = 1000.0;
  }

  int configure(std::string node_name, int hz);
  void initLTKF(Vec3 target_pos_wf);
  void resetLTKF(Vec3 target_pos_wf);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void quadVelocityCallback(const geometry_msgs::TwistStamped &msg);
  void targetWorldCallback(const geometry_msgs::Vector3 &msg);
  void publishLTKFInertialPositionEstimate(void);
  void publishLTKFInertialVelocityEstimate(void);
  void publishLTKFBodyPositionEstimate(void);
  void publishLTKFBodyVelocityEstimate(void);
  void publishLTDetected(void);
  void publishGimbalSetpointAttitudeMsg(Vec3 setpoints);
  void trackTarget(void);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
