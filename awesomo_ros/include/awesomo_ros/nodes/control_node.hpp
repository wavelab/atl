#ifndef __AWESOMO_ROS_NODES_CONTROL_NODE_HPP__
#define __AWESOMO_ROS_NODES_CONTROL_NODE_HPP__

#include <ros/ros.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <awesomo_msgs/AprilTagPose.h>
#include <awesomo_msgs/PCtrlStats.h>
#include <awesomo_msgs/PCtrlSettings.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_control"
#define NODE_RATE 100

// PUBLISH TOPICS
#define SETPOINT_ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define SETPOINT_THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"
#define SETPOINT_POSITION_TOPIC "/mavros/setpoint_position/local"
#define PCTRL_STATS_TOPIC "/awesomo/position_controller/stats"
#define PCTRL_GET_TOPIC "/awesomo/control/position_controller/get"
#define QUADROTOR_POSE "/awesomo/control/quad_pose"
#define ESTIMATOR_ON_TOPIC "/awesomo/estimator/on"
#define ESTIMATOR_OFF_TOPIC "/awesomo/estimator/off"

// SUBSCRIBE TOPICS
#define MODE_TOPIC "/mavros/set_mode"
#define ARM_TOPIC "/mavros/cmd/arming"
#define QMODE_TOPIC "/awesomo/control/mode"
#define STATE_TOPIC "/mavros/state"
#define POSE_TOPIC "/mavros/local_position/pose"
#define VELOCITY_TOPIC "/mavros/local_position/velocity"
#define HEADING_TOPIC "/awesomo/control/heading/set"
#define RADIO_TOPIC "/mavros/rc/in"
#define TARGET_BODY_POSITION_TOPIC "/awesomo/estimate/landing_target/position/body"
#define TARGET_BODY_VELOCITY_TOPIC "/awesomo/estimate/landing_target/velocity/body"
#define TARGET_DETECTED_TOPIC "/awesomo/estimate/landing_target/detected"
#define HOVER_SET_TOPIC "/awesomo/control/hover/set"
#define HOVER_HEIGHT_SET_TOPIC "/awesomo/control/hover/height/set"
#define PCTRL_SET_TOPIC "/awesomo/control/position_controller/set"
#define TCTRL_SET_TOPIC "/awesomo/control/tracking_controller/set"
#define LCTRL_SET_TOPIC "/awesomo/control/landing_controller/set"


class ControlNode : public ROSNode {
public:
  bool configured;

  Quadrotor quadrotor;
  std::string quad_frame;

  int rc_in[16];
  bool armed;

  mavros_msgs::State mavros_state;
  ros::ServiceClient mode_client;
  ros::ServiceClient arming_client;

  ControlNode(int argc, char **argv) : ROSNode(argc, argv) {
    for (int i = 0; i < 16; i++) {
      this->rc_in[i] = 0.0f;
    }
  }

  int configure(std::string node_name, int hz);
  void waitForFCU(void);
  void waitForEstimator(void);
  int disarm(void);
  int setOffboardModeOn(void);
  void setEstimatorOn(void);
  void setEstimatorOff(void);
  void modeCallback(const std_msgs::String &msg);
  void stateCallback(const mavros_msgs::State::ConstPtr &msg);
  void poseCallback(const geometry_msgs::PoseStamped &msg);
  void velocityCallback(const geometry_msgs::TwistStamped &msg);
  void headingCallback(const std_msgs::Float64 &msg);
  void radioCallback(const mavros_msgs::RCIn &msg);
  void targetPositionCallback(const geometry_msgs::Vector3 &msg);
  void targetVelocityCallback(const geometry_msgs::Vector3 &msg);
  void targetDetectedCallback(const std_msgs::Bool &msg);
  void hoverSetCallback(const geometry_msgs::Vector3 &msg);
  void hoverHeightSetCallback(const std_msgs::Float64 &msg);
  void positionControllerSetCallback(const awesomo_msgs::PCtrlSettings &msg);
  void trackingControllerSetCallback(const awesomo_msgs::TCtrlSettings &msg);
  void landingControllerSetCallback(const awesomo_msgs::LCtrlSettings &msg);
  int loopCallback(void);
  void publishStats(void);
};

}  // end of awesomo namespace
#endif
