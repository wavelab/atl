#ifndef ATL_MISSION_MISSION_HPP
#define ATL_MISSION_MISSION_HPP

#include <deque>

#include "atl/utils/utils.hpp"
#include "atl/mission/waypoint.hpp"

namespace atl {

// ERROR MESSAGES
#define EDISTLATLON "Waypoint %d has dist > %f from prev waypoint!"
#define EINVLATLON "Invalid latlon (%f, %f)!"
#define EINVALT "Invalid altitude %f!"
#define EINVSTAY "Invalid staytime %f!"
#define EINVHEADING \
  "Invalid heading %f! Should be between -180.0 to 180.0 degrees"

// CONSTANTS
#define CTRACK_HORIZ 0
#define CTRACK_3D 1

/**
 * Mission
 */
class Mission {
public:
  bool configured = false;

  double home_lat;
  double home_lon;
  double home_alt;

  bool check_waypoints = true;
  double threshold_waypoint_gap = 20.0;
  double threshold_waypoint_reached = 0.2;
  double desired_velocity = 0.5;
  double look_ahead_dist = 0.5;

  std::deque<Vec3> waypoints;
  Vec3 wp_start = Vec3::Zero();
  Vec3 wp_end = Vec3::Zero();

  Mission() {}

  /**
   * Configure
   * @param config_file Path to configuration fileN
   * @param home_lat Home point latitude
   * @param home_lon Home point longitude
   * @return
   *    - 0: success
   *    - -1: failure to load / parse configuration file
   *    - -2: invalid GPS waypoints
   */
  int configure(const std::string &config_file,
                double home_lat,
                double home_lon);

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

  /**
   * Calculate closest point
   *
   * @param position Actual position of robot
   * @return Closest point
   */
  Vec3 closestPoint(const Vec3 &position);

  /**
   * Calcuate which side the point is compared to waypoint track
   *
   * @param position Position
   * @return
   *    - 0: Position is colinear with waypoint track
   *    - 1: Position is left of waypoint track
   *    - -1: Position is right of waypoint track
   */
  int pointLineSide(const Vec3 &position);

  /**
   * Calculate waypoint point
   *
   * @param position Actual position of robot
   * @param r Lookahead distance in meters
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   */
  Vec3 waypointInterpolate(const Vec3 &position, const double r);

  /**
   * Calcuate cross track error
   *
   * @param position Position
   * @param mode Cross track error mode
   * @return Cross track error
   */
  double crossTrackError(const Vec3 &position, int mode = CTRACK_HORIZ);

  /**
   * Calculate waypoint tangent unit vector
   *
   * @return Unit tagent vector
   */
  Vec3 waypointTangentUnitVector();

  /**
   * Calculate waypoint yaw
   *
   * This function assumes we are operating in the NWU frame, where a 0
   * heading
   * starts from the x-axis and goes counter-clock-wise.
   *
   * @return Waypoint yaw
   */
  double waypointHeading();

  /**
   * Check whether waypoint is reached
   *
   * @param position Actual position of robot
   * @return
   *    - 0: Waypoint not reached
   *    - 1: Waypoint reached
   *    - -1: Not configured
   */
  int waypointReached(const Vec3 &position);

  /**
   * Update waypoint
   *
   * @param position Actual position of robot
   * @param waypoint Waypoint to update
   * @return
   *   - 0: Success
   *   - -1: Not configured
   *   - -2: No more waypoints
   */
  int update(const Vec3 &position, Vec3 &waypoint);
};

}  // namespace atl

#endif
