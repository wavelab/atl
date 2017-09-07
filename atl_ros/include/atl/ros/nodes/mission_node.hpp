#ifndef ATL_ROS_NODES_MISSION_NODE_HPP
#define ATL_ROS_NODES_MISSION_NODE_HPP

#include <ros/ros.h>


// NODE SETTINGS
static const double NODE_RATE = 100;

// PUBLISH TOPICS


namespace atl {

class MissionNode : public ROSNode {
public:
  std::vector<Vec3> gps_waypoints;
  double threshold_waypoint_gap = 20.0;

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
   * Load mission
   *
   * @param task Mission waypoint task
   * @return 0 for success, -1 for failure
   */
  int loadMission();
};

} // namespace atl
#endif
