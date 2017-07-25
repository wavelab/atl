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
  DJIDrone *dji = nullptr;
  bool offboard = false;

  MissionNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~MissionNode() {
    delete this->dji;
  }

  /**
   * Configure
   * @params node_name ROS node name
   * @params hz ROS node rate
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &node_name, int hz);

  /**
   * ROS R/C radio callback
   */
  void djiRadioCallback(const dji_sdk::RCChannels &msg);

  /**
   * Configure mode on
   * @returns 0 for success, -1 for failure
   */
  int djiOffboardModeOn();

  /**
   * Offboard mode off
   * @returns 0 for success, -1 for failure
   */
  int djiOffboardModeOff();

  /**
   * Run mission
   * @returns 0 for success, -1 for failure
   */
  int runMission();
};

}  // namespace atl
#endif
