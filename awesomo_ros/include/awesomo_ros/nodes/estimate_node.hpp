#ifndef __AWESOMO_ROS_NODES_ESTIMATE_NODE_HPP__
#define __AWESOMO_ROS_NODES_ESTIMATE_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"

namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_estimate"
#define NODE_RATE 1000

// PUBLISH TOPICS
#define LT_WORLD_TOPIC "/awesomo/estimate/landing_target/world"
#define LT_LOCAL_TOPIC "/awesomo/estimate/landing_target/local"

// SUBSCRIBE TOPICS
#define GIMBAL_TARGET_TOPIC "/awesomo/gimbal/target"
#define LT_INIT_TOPIC "/awesomo/estimate/landing_target/init"

class EstimateNode : public ROSNode {
public:
  KalmanFilter lt_kf;

  Pose quad_pose;
  bool target_detected;
  Vec3 target_bpf;

  EstimateNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  void gimbalTargetCallback(const geometry_msgs::Vector3 &msg);
  void initLTKFCallback(const std_msgs::Bool &msg);
  void publishLTKFWorldEstimate(void);
  void publishLTKFLocalEstimate(void);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
