#ifndef ATL_MISSION_MISSION_HPP
#define ATL_MISSION_MISSION_HPP

#include "atl/utils/utils.hpp"

namespace atl {

// ERROR MESSAGES
#define EDISTLATLON "Waypoint %d: (%f, %f) has dist > %f from prev waypoint!"

/**
 * Waypoint
 */
class Waypoint {
public:
  double latitude;
  double longitude;

  Waypoint() : latitude{0.0}, longitude{0.0} {}
  Waypoint(double latitude, double longitude)
      : latitude{latitude}, longitude{longitude} {}

  /**
   * Calculate distance away from another waypoint
   * @param wp 2nd Waypoint to calculate distance away from
   * @return Distance between this waypoint and waypoint `wp`
   */
  double distance(const Waypoint &wp) {
    return latlon_dist(latitude, longitude, wp.latitude, wp.longitude);
  }

  friend std::ostream &operator<<(std::ostream &out, const Waypoint &wp) {
    out << "latitude:" << wp.latitude << ", ";
    out << "longitude:" << wp.longitude;
    return out;
  }
};

/**
 * Mission
 */
class Mission {
public:
  bool configured;

  bool check_waypoints;
  double waypoint_threshold;

  double altitude;
  double max_velocity;
  std::vector<Waypoint> waypoints;

  Mission()
      : configured{false},
        check_waypoints{true},
        waypoint_threshold{20.0},
        altitude{5.0},
        max_velocity{0.5} {}

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
