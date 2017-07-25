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
   * Build DJI waypoint list
   * @return Mission waypoint list
   */
  dji_sdk::WaypointList buildMission();

  /**
   * ROS R/C radio callback
   */
  void radioCallback(const dji_sdk::RCChannels &msg);

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
