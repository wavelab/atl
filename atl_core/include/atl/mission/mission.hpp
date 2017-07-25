#ifndef ATL_MISSION_MISSION_HPP
#define ATL_MISSION_MISSION_HPP

#include "atl/utils/utils.hpp"
#include "atl/mission/waypoint.hpp"

namespace atl {

// ERROR MESSAGES
#define EDISTLATLON "Waypoint %d: (%f, %f) has dist > %f from prev waypoint!"
#define EINVLATLON "Invalid latlon (%f, %f)!"
#define EINVALT "Invalid altitude %f!"
#define EINVSTAY "Invalid staytime %f!"
#define EINVHEADING \
  "Invalid heading %f! Should be between -180.0 to 180.0 degrees"

/**
 * Mission
 */
class Mission {
public:
  bool configured;

  bool check_waypoints;
  double waypoint_threshold;

  double velocity;
  std::vector<Waypoint> waypoints;

  Mission()
      : configured{false},
        check_waypoints{true},
        waypoint_threshold{20.0},
        velocity{0.0} {}

  /**
   * Configure
   * @param config_file Path to configuration file
   * @return
   *    - 0: success
   *    - -1: failure to load / parse configuration file
   *    - -2: invalid GPS waypoints
   */
  int configure(const std::string &config_file);

  /**
   * Check waypoints
   *
   * Make sure the distance between waypoints does not exceed
   * `Mission.waypoint_threshold`
   *
   * @param config_file Path to configuration file
   * @return
   *    - 0: success
   *    - -1: no waypoints loaded
   *    - -2: invalid GPS waypoints
   */
  int checkWaypoints();
};

}  // namespace atl

#endif
