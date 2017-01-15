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
#define LT_INERTIAL_TOPIC "/awesomo/estimate/landing_target/inertial"
#define LT_BODY_TOPIC "/awesomo/estimate/landing_target/body"
#define LT_VELOCITY_TOPIC "/awesomo/estimate/landing_target/velocity"
#define GIMBAL_SETPOINT_ATTITUDE_TOPIC "/awesomo/gimbal/setpoint/attitude"

// SUBSCRIBE TOPICS
#define QUAD_POSE_TOPIC "/mavros/local_position/pose"
#define TARGET_INERTIAL_TOPIC "/awesomo/apriltag/target/inertial"

class EstimatorNode : public ROSNode {
public:
  KalmanFilter lt_kf;

  Pose quad_pose;
  bool target_detected;
  Vec3 target_bpf;
  Vec3 target_wpf;
  Vec3 target_last_wpf;
  struct timespec target_last_updated;
  double target_lost_threshold;

  EstimatorNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->lt_kf = KalmanFilter();

    this->quad_pose = Pose();
    this->target_detected = false;
    this->target_bpf << 0.0, 0.0, 0.0;
    this->target_wpf << 0.0, 0.0, 0.0;
    this->target_last_wpf << 0.0, 0.0, 0.0;
    this->target_last_updated = (struct timespec){0};
    this->target_lost_threshold = 1000.0;
  }

  int configure(std::string node_name, int hz);
  void initLTKF(Vec3 target_wpf);
  void resetLTKF(Vec3 target_wpf);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void targetWorldCallback(const geometry_msgs::Vector3 &msg);
  void publishLTKFWorldEstimate(void);
  void publishLTKFLocalEstimate(void);
  void publishLTKFVelocityEstimate(void);
  void publishGimbalSetpointAttitudeMsg(Vec3 setpoints);
  void trackTarget(void);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
