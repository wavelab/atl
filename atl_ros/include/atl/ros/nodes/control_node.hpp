#ifndef ATL_ROS_NODES_CONTROL_NODE_HPP
#define ATL_ROS_NODES_CONTROL_NODE_HPP

#include <ros/ros.h>

#define MAVLINK_DIALECT common
#include <mavros/mavros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <dji_sdk/dji_drone.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/SDKControlAuthority.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

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

#define DJI_ARM_TOPIC "/dji_sdk/drone_arm_control"
#define DJI_SDK_AUTH_TOPIC "/dji_sdk/sdk_control_authority"
#define DJI_SETPOINT_TOPIC "/dji_sdk/flight_control_setpoint_generic"
#define DJI_GPS_POSITION_TOPIC "/dji_sdk/global_position"
#define DJI_LOCAL_POSITION_TOPIC "/dji_sdk/local_position"
#define DJI_ATTITUDE_TOPIC "/dji_sdk/attitude_quaternion"
#define DJI_VELOCITY_TOPIC "/dji_sdk/velocity"
#define DJI_RADIO_TOPIC "/dji_sdk/rc"

#define ARM_TOPIC "/atl/control/arm"
#define MODE_TOPIC "/atl/control/mode"
#define YAW_TOPIC "/atl/control/yaw/set"
#define TARGET_BODY_POSITION_TOPIC \
  "/atl/estimate/landing_target/position/body"
#define TARGET_BODY_VELOCITY_TOPIC \
  "/atl/estimate/landing_target/velocity/body"
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
  DJIDrone *dji;

  Quadrotor quadrotor;
  bool armed;
  double latitude;
  double longitude;

  ControlNode(int argc, char **argv)
      : ROSNode(argc, argv),
        configured{false},
        fcu_type{},
        quad_frame{},
        px4_state{},
        dji{nullptr},
        quadrotor{},
        armed{false},
        latitude{},
        longitude{} {}

  int configure(std::string node_name, int hz);
  int configurePX4Topics();
  int configureDJITopics();
  int px4Connect();
  int px4Disarm();
  int px4OffboardModeOn();
  int djiDisarm();
  int djiOffboardModeOn();
  int djiOffboardModeOff();
  int waitForEstimator();
  void setEstimatorOn();
  void setEstimatorOff();
  void px4StateCallback(const mavros_msgs::State::ConstPtr &msg);
  void px4PoseCallback(const geometry_msgs::PoseStamped &msg);
  void px4VelocityCallback(const geometry_msgs::TwistStamped &msg);
  void px4RadioCallback(const mavros_msgs::RCIn &msg);
  void djiGPSPositionCallback(const sensor_msgs::NavSatFix &msg);
  void djiLocalPositionCallback(const dji_sdk::LocalPosition &msg);
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
  void publishAttitudeSetpoint();
  void publishQuadrotorPose();
  void publishQuadrotorVelocity();
  int loopCallback();
};

}  // namespace atl
#endif
