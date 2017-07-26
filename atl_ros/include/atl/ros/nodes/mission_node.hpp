#ifndef ATL_ROS_NODES_MISSION_NODE_HPP
#define ATL_ROS_NODES_MISSION_NODE_HPP

#include <ros/ros.h>

#include <dji_sdk/dji_drone.h>

#include <atl/atl_core.hpp>

#include "atl/ros/utils/msgs.hpp"
#include "atl/ros/utils/node.hpp"

namespace atl {

// NODE SETTINGS
#define NODE_NAME "atl_mission"
#define NODE_RATE 10

// PUBLISH TOPICS
#define DJI_GPS_POSITION_TOPIC "/dji_sdk/global_position"
#define DJI_LOCAL_POSITION_TOPIC "/dji_sdk/local_position"
#define DJI_ATTITUDE_TOPIC "/dji_sdk/attitude_quaternion"
#define DJI_VELOCITY_TOPIC "/dji_sdk/velocity"
#define DJI_RADIO_TOPIC "/dji_sdk/rc_channels"

class MissionNode : public ROSNode {
public:
  bool offboard;

  DJIDrone *dji;
  Mission mission;

  MissionNode() : offboard{false}, dji{nullptr}, mission{} {}
  MissionNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~MissionNode() {
    delete this->dji;
  }

  /**
   * Configure
   * @param node_name ROS node name
   * @param hz ROS node rate
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &node_name, int hz);

  /**
   * GPS position callback
   * @param msg ROS message
   */
  void gpsPositionCallback(const dji_sdk::GlobalPosition &msg);

  /**
   * Local position callback
   * @param msg ROS message
   */
  void localPositionCallback(const dji_sdk::LocalPosition &msg);

  /**
   * Attitude callback
   * @param msg ROS message
   */
  void attitudeCallback(const dji_sdk::AttitudeQuaternion &msg);

  /**
   * Velocity callback
   * @param msg ROS message
   */
  void velocityCallback(const dji_sdk::Velocity &msg);

  /**
   * Radio callback
   * @param msg ROS message
   */
  void radioCallback(const dji_sdk::RCChannels &msg);

  /**
   * Takeoff
   * @return
   *    - 0: Success
   *    - -1: ROS node not configured
   *    - -2: Not in offboard mode
   */
  int takeoff();

  /**
   * Land
   * @return
   *    - 0: Success
   *    - -1: ROS node not configured
   *    - -2: Not in offboard mode
   */
  int land();

  /**
   * Configure mode on
   * @return 0 for success, -1 for failure
   */
  int offboardModeOn();

  /**
   * Offboard mode off
   * @return 0 for success, -1 for failure
   */
  int offboardModeOff();

  /**
   * Execute mission
   * @return 0 for success, -1 for failure
   */
  int executeMission();
};

}  // namespace atl
#endif
