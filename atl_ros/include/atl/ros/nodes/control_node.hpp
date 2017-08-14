#ifndef ATL_ROS_NODES_CONTROL_NODE_HPP
#define ATL_ROS_NODES_CONTROL_NODE_HPP

#include <ros/ros.h>

#include <dji_sdk/dji_drone.h>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"
#include <atl/atl_core.hpp>

namespace atl {

// NODE SETTINGS
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

  ~ControlNode() {
    if (this->dji != nullptr) {
      delete this->dji;
    }
  }

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(int hz);

  /**
   * Disarm
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int disarm();

  /**
   * SDK control mode
   *
   * @param mode Switch offboard mode on/off
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int sdkControlMode(const bool mode);

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
   * GPS position callback
   * @param msg ROS message
   */
  void gpsPositionCallback(const dji_sdk::GlobalPosition &msg);

  /**
   * local position callback
   * @param msg ROS message
   */
  void localPositionCallback(const dji_sdk::LocalPosition &msg);

  /**
   * attitude callback
   * @param msg ROS message
   */
  void attitudeCallback(const dji_sdk::AttitudeQuaternion &msg);

  /**
   * velocity callback
   * @param msg ROS message
   */
  void velocityCallback(const dji_sdk::Velocity &msg);

  /**
   * Radio callback
   * @param msg ROS message
   */
  void radioCallback(const dji_sdk::RCChannels &msg);

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

} // namespace atl
#endif
