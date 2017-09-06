#ifndef ATL_ROS_NODES_CONTROL_NODE_HPP
#define ATL_ROS_NODES_CONTROL_NODE_HPP

#include <ros/ros.h>

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"
#include <atl/atl_core.hpp>

// NODE SETTINGS
static const double NODE_RATE = 50;

// clang-format off
// PUBLISH TOPICS
static const std::string PCTRL_STATS_TOPIC = "/atl/position_controller/stats";
static const std::string PCTRL_GET_TOPIC = "/atl/control/position_controller/get";
static const std::string QUADROTOR_POSE_TOPIC = "/atl/quadrotor/pose/local";
static const std::string QUADROTOR_VELOCITY_TOPIC = "/atl/quadrotor/velocity/local";
static const std::string ESTIMATOR_ON_TOPIC = "/atl/estimator/on";
static const std::string ESTIMATOR_OFF_TOPIC = "/atl/estimator/off";
static const std::string DJI_CONTROL_TOPIC = "/dji_sdk/flight_control_setpoint_generic";

// SUBSCRIBE TOPICS
static const std::string DJI_GPS_POSITION_TOPIC = "/dji_sdk/gps_position";
static const std::string DJI_ATTITUDE_TOPIC = "/dji_sdk/attitude";
static const std::string DJI_VELOCITY_TOPIC = "/dji_sdk/velocity";
static const std::string DJI_RADIO_TOPIC = "/dji_sdk/rc";

static const std::string ARM_TOPIC = "/atl/control/arm";
static const std::string MODE_TOPIC = "/atl/control/mode";
static const std::string YAW_TOPIC = "/atl/control/yaw/set";
static const std::string TARGET_BODY_POSITION_TOPIC = "/atl/estimate/landing_target/position/body";
static const std::string TARGET_BODY_VELOCITY_TOPIC = "/atl/estimate/landing_target/velocity/body";
static const std::string TARGET_DETECTED_TOPIC = "/atl/estimate/landing_target/detected";
static const std::string HOVER_SET_TOPIC = "/atl/control/hover/set";
static const std::string HOVER_HEIGHT_SET_TOPIC = "/atl/control/hover/height/set";
static const std::string PCTRL_SET_TOPIC = "/atl/control/position_controller/set";
static const std::string TCTRL_SET_TOPIC = "/atl/control/tracking_controller/set";
static const std::string LCTRL_SET_TOPIC = "/atl/control/landing_controller/set";
// clang-format on

// SERVICE TOPICS
static const std::string DJI_SDK_SERVICE = "/dji_sdk/sdk_control_authority";
static const std::string DJI_ARM_SERVICE = "/dji_sdk/drone_arm_service";

namespace atl {

class ControlNode : public ROSNode {
public:
  bool configured = false;

  Quadrotor quadrotor;
  bool armed = false;
  int gps_status = -1;
  int gps_service = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  bool home_set = false;
  double home_latitude = 0.0;
  double home_longitude = 0.0;
  double home_altitude = 0.0;

  ControlNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~ControlNode() {}

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(const int hz);

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
   * Global position callback
   * @param msg ROS message
   */
  void globalPositionCallback(const sensor_msgs::NavSatFix &msg);

  /**
   * attitude callback
   * @param msg ROS message
   */
  void attitudeCallback(const geometry_msgs::QuaternionStamped &msg);

  /**
   * velocity callback
   * @param msg ROS message
   */
  void velocityCallback(const geometry_msgs::Vector3Stamped &msg);

  /**
   * Radio callback
   * @param msg ROS message
   */
  void radioCallback(const sensor_msgs::Joy &msg);

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
