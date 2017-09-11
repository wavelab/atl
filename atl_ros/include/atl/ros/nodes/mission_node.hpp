#ifndef ATL_ROS_NODES_MISSION_NODE_HPP
#define ATL_ROS_NODES_MISSION_NODE_HPP

#include <ros/ros.h>

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <djiosdk/dji_vehicle.hpp>

#include "atl/atl_core.hpp"
#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

// ERROR MESSAGES
// clang-format off
#define EINVWPDATA "Invalid number of waypoint data!"
#define EDISTLATLON "Waypoint %d: (%f, %f) has dist > %f from prev waypoint!"
#define EINVLATLON "Invalid latlon (%f, %f)!"
#define EINVALT "Invalid altitude %f!"
#define EINVSTAY "Invalid staytime %f!"
#define EINVHEADING "Invalid heading %f! Should be between -180.0 to 180.0 degrees"
#define EDJISDKREQ "Failed to request DJI SDK control!"
#define EDJISDKREL "Failed to release DJI SDK control!"
#define EMISSIONLOAD "Failed to load mission!"
#define EMISSIONUP "Failed to upload mission!"
#define EMISSIONSTART "Failed to start mission!"
#define EMISSIONSTOP "Failed to stop mission!"
// clang-format on

// INFO MESSAGES
#define INFO_DJI_SDK_OBTAINED "Obtained DJI SDK control!"
#define INFO_DJI_SDK_RELEASED "Released DJI SDK control!"

// NODE SETTINGS
static const double NODE_RATE = 100;

// PUBLISH TOPICS
static const std::string DJI_PCTRL_TOPIC = "/dji_sdk/flight_control_setpoint_ENUposition_yaw";

// SUBSCRIBE TOPICS
static const std::string DJI_RADIO_TOPIC = "/dji_sdk/rc";
static const std::string DJI_FLIGHT_STATUS_TOPIC = "/dji_sdk/flight_status";
static const std::string DJI_DISPLAY_MODE_TOPIC = "/dji_sdk/display_mode";

// SERVICE TOPICS
// clang-format off
static const std::string DJI_SDK_SERVICE = "/dji_sdk/sdk_control_authority";
static const std::string DJI_ARM_SERVICE = "/dji_sdk/drone_arm_control";
static const std::string DJI_TASK_SERVICE = "/dji_sdk/drone_task_control";
static const std::string DJI_WAYPOINT_UPLOAD_SERVICE = "/dji_sdk/mission_waypoint_upload";
static const std::string DJI_WAYPOINT_ACTION_SERVICE = "/dji_sdk/mission_waypoint_action";
// clang-format on

namespace atl {

enum MISSION_STATE {
  MISSION_IDEL = -1,
  MISSION_RUNNING = 1,
  MISSION_COMPLETED = 2
};

class MissionNode : public ROSNode {
public:
  double desired_velocity = 5.0;
  double threshold_waypoint_gap = 30.0;
  double mission_yaw = 0.0;
  int yaw_mode = 0;
  double exec_times = 1.0;
  std::vector<Vec3> gps_waypoints;

  int state = MISSION_IDEL;
  uint8_t flight_status = 255;
  uint8_t display_mode = 255;

  MissionNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure ROS node
   *
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(const int hz);

  /**
   * SDK control mode
   *
   * @param mode Switch offboard mode on/off
   * @return 0 for success, -1 for failure
   */
  int sdkControlMode(const bool mode);

  /**
   * Arm quadrotor
   * @return 0 for success, -1 for failure
   */
  int arm();

  /**
   * Disarm quadrotor
   * @return 0 for success, -1 for failure
   */
  int disarm();

  /**
   * Radio callback
   * @param msg ROS message
   */
  void radioCallback(const sensor_msgs::Joy &msg);

  /**
   * Flight status callback
   * @param msg ROS message
   */
  void flightStatusCallback(const std_msgs::UInt8 &msg);

  /**
   * Display mode callback
   * @param msg ROS message
   */
  void displayModeCallback(const std_msgs::UInt8 &msg);

  /**
   * Load mission
   *
   * @param config_file Configuration file
   * @return 0 for success, -1 for failure
   */
  int loadMission(const std::string &config_file);

  /**
   * Upload mission
   * @return 0 for success, -1 for failure
   */
  int uploadMission();

  /**
   * Start mission
   * @return 0 for success, -1 for failure
   */
  int startMission();

  /**
   * Stop mission
   * @return 0 for success, -1 for failure
   */
  int stopMission();
};

} // namespace atl
#endif
