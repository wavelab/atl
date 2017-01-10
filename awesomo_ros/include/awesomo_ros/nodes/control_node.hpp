#ifndef __AWESOMO_ROS_NODES_CONTROL_NODE_HPP__
#define __AWESOMO_ROS_NODES_CONTROL_NODE_HPP__

#include <ros/ros.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <awesomo_msgs/KFStats.h>
#include <awesomo_msgs/KFPlot.h>
#include <awesomo_msgs/AprilTagPose.h>
#include <awesomo_msgs/PCtrlStats.h>
#include <awesomo_msgs/PCtrlSettings.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define CONTROL_NODE_NAME "awesomo_control"
#define CONTROL_NODE_RATE 100

// PUBLISH TOPICS
#define PCTRL_STATS_TOPIC "/awesomo/position_controller/stats"
#define KF_STATS_TOPIC "/awesomo/kf_estimation/stats"
#define KF_PLOT_TOPIC "/awesomo/kf_estimation/plot"
#define PCTRL_GET_TOPIC "/awesomo/control/position_controller/get"

// SUBSCRIBE TOPICS
#define MODE_TOPIC "/mavros/set_mode"
#define ARM_TOPIC "/mavros/cmd/arming"
#define SHUTDOWN "/awesomo/control/shutdown"
#define STATE_TOPIC "/mavros/state"
#define POSE_TOPIC "/mavros/local_position/pose"
#define RADIO_TOPIC "/mavros/rc/in"
#define APRILTAG_TOPIC "/awesomo/apriltag/pose"
#define TARGET_TOPIC "/awesomo/gimbal/target"
#define SETPOINT_ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define SETPOINT_THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"
#define SETPOINT_POSITION_TOPIC "/mavros/setpoint_position/local"
#define HOVER_SET_TOPIC "/awesomo/control/hover/set"
#define PCTRL_SET_TOPIC "/awesomo/control/position_controller/set"

class ControlNode : public ROSNode {
public:
  bool configured;

  Pose world_pose;
  TagPose tag_pose;
  Vec3 target_bpf;
  int rc_in[16];
  Quadrotor quadrotor;

  mavros_msgs::State mavros_state;
  ros::ServiceClient mode_client;
  ros::ServiceClient arming_client;

  ControlNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->world_pose = Pose();
    this->tag_pose = TagPose();
    this->target_bpf = Vec3();

    for (int i = 0; i < 16; i++) {
      this->rc_in[i] = 0.0f;
    }
  }

  int configure(std::string node_name, int hz);
  void waitForConnection(void);
  int disarm(void);
  int setOffboardModeOn(void);
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void poseCallback(const geometry_msgs::PoseStamped &msg);
  void aprilTagCallback(const awesomo_msgs::AprilTagPose &msg);
  void targetCallback(const geometry_msgs::Vector3 &msg);
  void radioCallback(const mavros_msgs::RCIn &msg);
  void hoverSetCallback(const geometry_msgs::Vector3 &msg);
  void positionControllerSetCallback(const awesomo_msgs::PCtrlSettings &msg);
  int loopCallback(void);
  void publishStats(void);
};

}  // end of awesomo namespace
#endif
