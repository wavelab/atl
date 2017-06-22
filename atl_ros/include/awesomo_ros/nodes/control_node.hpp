#ifndef ATL_ROS_NODES_CONTROL_NODE_HPP
#define ATL_ROS_NODES_CONTROL_NODE_HPP

#include <ros/ros.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>

#include <dji_sdk/dji_drone.h>

#include <atl_msgs/AprilTagPose.h>
#include <atl_msgs/PCtrlStats.h>
#include <atl_msgs/PCtrlSettings.h>

#include <atl_core/atl_core.hpp>

#include "atl_ros/utils/node.hpp"
#include "atl_ros/utils/msgs.hpp"


namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_control"
#define NODE_RATE 50

// PUBLISH TOPICS
#define PX4_SETPOINT_ATTITUDE_TOPIC "/mavros/setpoint_attitude/attitude"
#define PX4_SETPOINT_THROTTLE_TOPIC "/mavros/setpoint_attitude/att_throttle"
#define PX4_SETPOINT_POSITION_TOPIC "/mavros/setpoint_position/local"

#define PCTRL_STATS_TOPIC "/atl/position_controller/stats"
#define PCTRL_GET_TOPIC "/atl/control/position_controller/get"
#define QUADROTOR_POSE "/atl/quadrotor/pose/local"
#define QUADROTOR_VELOCITY "/atl/quadrotor/velocity/local"
#define ESTIMATOR_ON_TOPIC "/atl/estimator/on"
#define ESTIMATOR_OFF_TOPIC "/atl/estimator/off"

// SUBSCRIBE TOPICS
#define PX4_MODE_TOPIC "/mavros/set_mode"
#define PX4_ARM_TOPIC "/mavros/cmd/arming"
#define PX4_STATE_TOPIC "/mavros/state"
#define PX4_POSE_TOPIC "/mavros/local_position/pose"
#define PX4_VELOCITY_TOPIC "/mavros/local_position/velocity"
#define PX4_RADIO_TOPIC "/mavros/rc/in"

#define DJI_POSITION_TOPIC "/dji_sdk/local_position"
#define DJI_ATTITUDE_TOPIC "/dji_sdk/attitude_quaternion"
#define DJI_VELOCITY_TOPIC "/dji_sdk/velocity"
#define DJI_RADIO_TOPIC "/dji_sdk/rc_channels"

#define ARM_TOPIC "/atl/control/arm"
#define MODE_TOPIC "/atl/control/mode"
#define YAW_TOPIC "/atl/control/yaw/set"
#define TARGET_BODY_POSITION_TOPIC "/atl/estimate/landing_target/position/body"
#define TARGET_BODY_VELOCITY_TOPIC "/atl/estimate/landing_target/velocity/body"
#define TARGET_DETECTED_TOPIC "/atl/estimate/landing_target/detected"
#define HOVER_SET_TOPIC "/atl/control/hover/set"
#define HOVER_HEIGHT_SET_TOPIC "/atl/control/hover/height/set"
#define PCTRL_SET_TOPIC "/atl/control/position_controller/set"
#define TCTRL_SET_TOPIC "/atl/control/tracking_controller/set"
#define LCTRL_SET_TOPIC "/atl/control/landing_controller/set"


class ControlNode : public ROSNode {
public:
  bool configured;

  std::string quad_frame;
  std::string fcu_type;

  mavros_msgs::State px4_state;
  ros::ServiceClient px4_mode_client;
  ros::ServiceClient px4_arming_client;

  DJIDrone *dji;

  Quadrotor quadrotor;
  int rc_in[16];
  bool armed;

  ControlNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->quad_frame = "";
    this->fcu_type = "";

    this->dji = NULL;

    for (int i = 0; i < 16; i++) {
      this->rc_in[i] = 0.0f;
    }
    this->armed = false;
  }

  int configure(std::string node_name, int hz);
  int configurePX4Topics(void);
  int configureDJITopics(void);
  int px4Connect(void);
  int px4Disarm(void);
  int px4OffboardModeOn(void);
  int djiDisarm(void);
  int djiOffboardModeOn(void);
  int djiOffboardModeOff(void);
  int waitForEstimator(void);
  void setEstimatorOn(void);
  void setEstimatorOff(void);
  void px4StateCallback(const mavros_msgs::State::ConstPtr &msg);
  void px4PoseCallback(const geometry_msgs::PoseStamped &msg);
  void px4VelocityCallback(const geometry_msgs::TwistStamped &msg);
  void px4RadioCallback(const mavros_msgs::RCIn &msg);
  void djiPositionCallback(const dji_sdk::LocalPosition &msg);
  void djiAttitudeCallback(const dji_sdk::AttitudeQuaternion &msg);
  void djiVelocityCallback(const dji_sdk::Velocity &msg);
  void djiRadioCallback(const dji_sdk::RCChannels &msg);
  void armCallback(const std_msgs::Bool &msg);
  void modeCallback(const std_msgs::String &msg);
  void yawCallback(const std_msgs::Float64 &msg);
  void targetPositionCallback(const geometry_msgs::Vector3 &msg);
  void targetVelocityCallback(const geometry_msgs::Vector3 &msg);
  void targetDetectedCallback(const std_msgs::Bool &msg);
  void hoverSetCallback(const geometry_msgs::Vector3 &msg);
  void hoverHeightSetCallback(const std_msgs::Float64 &msg);
  void positionControllerSetCallback(const atl_msgs::PCtrlSettings &msg);
  void trackingControllerSetCallback(const atl_msgs::TCtrlSettings &msg);
  void landingControllerSetCallback(const atl_msgs::LCtrlSettings &msg);
  void publishAttitudeSetpoint(void);
  void publishQuadrotorPose(void);
  void publishQuadrotorVelocity(void);
  void publishPX4DummyMsg(void);
  int loopCallback(void);
  void publishStats(void);
};

}  // end of atl namespace
#endif
