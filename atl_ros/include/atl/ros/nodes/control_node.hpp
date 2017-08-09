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
// clang-format off
#define PX4_MODE_TOPIC "/mavros/set_mode"
#define PX4_ARM_TOPIC "/mavros/cmd/arming"
#define PX4_STATE_TOPIC "/mavros/state"
#define PX4_POSE_TOPIC "/mavros/local_position/pose"
#define PX4_VELOCITY_TOPIC "/mavros/local_position/velocity"
#define PX4_RADIO_TOPIC "/mavros/rc/in"

#define DJI_GPS_POSITION_TOPIC "/dji_sdk/global_position"
#define DJI_LOCAL_POSITION_TOPIC "/dji_sdk/local_position"
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
// clang-format on

class ControlNode : public ROSNode {
public:
  bool configured = false;

  std::string quad_frame = "";
  std::string fcu_type = "";

  mavros_msgs::State px4_state;
  DJIDrone *dji = nullptr;

  Quadrotor quadrotor;
  bool armed = false;
  double latitude = 0.0;
  double longitude = 0.0;
  bool home_set = false;
  double home_latitude = 0.0;
  double home_longitude = 0.0;
  double home_altitude = 0.0;

  ControlNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(int hz);

  /**
   * Configure PX4 ROS topics
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int configurePX4Topics();

  /**
   * Configure DJI ROS topics
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int configureDJITopics();

  /**
   * PX4 connect
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int px4Connect();

  /**
   * PX4 disarm
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int px4Disarm();

  /**
   * PX4 offboard mode on
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int px4OffboardModeOn();

  /**
   * DJI disarm
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int djiDisarm();

  /**
   * DJI offboard mode on
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int djiOffboardModeOn();

  /**
   * DJI offboard mode off
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int djiOffboardModeOff();

  /**
   * Wait for estimator connection
   *
   * @return
   *    - 0: Success
   *    - -1: No connection to estimator
   */
  int waitForEstimator();

  /**
   * Wait for GPS connection
   *
   * @return
   *    - 0: Success
   *    - -1: No connection to GPS
   */
  int waitForGPS();

  /**
   * Estimator on
   */
  void setEstimatorOn();

  /**
   * Estimator off
   */
  void setEstimatorOff();

  /**
   * PX4 state callback
   * @param msg ROS message
   */
  void px4StateCallback(const mavros_msgs::State::ConstPtr &msg);

  /**
   * PX4 pose callback
   * @param msg ROS message
   */
  void px4PoseCallback(const geometry_msgs::PoseStamped &msg);

  /**
   * PX4 velocity callback
   * @param msg ROS message
   */
  void px4VelocityCallback(const geometry_msgs::TwistStamped &msg);

  /**
   * PX4 radio callback
   * @param msg ROS message
   */
  void px4RadioCallback(const mavros_msgs::RCIn &msg);

  /**
   * DJI GPS position callback
   * @param msg ROS message
   */
  void djiGPSPositionCallback(const dji_sdk::GlobalPosition &msg);

  /**
   * DJI local position callback
   * @param msg ROS message
   */
  void djiLocalPositionCallback(const dji_sdk::LocalPosition &msg);

  /**
   * DJI attitude callback
   * @param msg ROS message
   */
  void djiAttitudeCallback(const dji_sdk::AttitudeQuaternion &msg);

  /**
   * DJI velocity callback
   * @param msg ROS message
   */
  void djiVelocityCallback(const dji_sdk::Velocity &msg);

  /**
   * DJI Radio callback
   * @param msg ROS message
   */
  void djiRadioCallback(const dji_sdk::RCChannels &msg);

  /**
   * Arm callback
   * @param msg ROS message
   */
  void armCallback(const std_msgs::Bool &msg);

  /**
   * Mode callback
   * @param msg ROS message
   */
  void modeCallback(const std_msgs::String &msg);

  /**
   * Yaw callback
   * @param msg ROS message
   */
  void yawCallback(const std_msgs::Float64 &msg);

  /**
   * Target position callback
   * @param msg ROS message
   */
  void targetPositionCallback(const geometry_msgs::Vector3 &msg);

  /**
   * Target velocity callback
   * @param msg ROS message
   */
  void targetVelocityCallback(const geometry_msgs::Vector3 &msg);

  /**
   * Target detected callback
   * @param msg ROS message
   */
  void targetDetectedCallback(const std_msgs::Bool &msg);

  /**
   * Hover setpoint callback
   * @param msg ROS message
   */
  void hoverSetCallback(const geometry_msgs::Vector3 &msg);

  /**
   * Hover height setpoint callback
   * @param msg ROS message
   */
  void hoverHeightSetCallback(const std_msgs::Float64 &msg);

  /**
   * Position controller callback
   * @param msg ROS message
   */
  void positionControllerSetCallback(const atl_msgs::PCtrlSettings &msg);

  /**
   * Tracking controller callback
   * @param msg ROS message
   */
  void trackingControllerSetCallback(const atl_msgs::TCtrlSettings &msg);

  /**
   * Landing controller callback
   * @param msg ROS message
   */
  void landingControllerSetCallback(const atl_msgs::LCtrlSettings &msg);

  /**
   * Publish attitude setpoints
   */
  void publishAttitudeSetpoint();

  /**
   * Publish quadrotor pose
   */
  void publishQuadrotorPose();

  /**
   * Publish quadrotor velocity
   */
  void publishQuadrotorVelocity();

  /**
   * Takeoff
   *
   * @return
   *    - 0: Success
   *    - -1: ROS node not configured
   *    - -2: Not in offboard mode
   */
  int takeoff();

  /**
   * Land
   *
   * @return
   *    - 0: Success
   *    - -1: ROS node not configured
   *    - -2: Not in offboard mode
   */
  int land();

  /**
   * ROS node loop function
   *
   * @return
   *    - 0: Continue looping
   *    - -1: Stop looping
   */
  int loopCallback();
};

}  // namespace atl
#endif
