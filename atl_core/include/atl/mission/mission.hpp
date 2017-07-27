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

// CONSTANTS
#define CTRACK_HORIZ 0
#define CTRACK_3D 1

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
  Vec3 wp_start = Vec3::Zero();
  Vec3 wp_end = Vec3::Zero();

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

  /**
   * Calculate closest point
   *
   * @param position Actual position of robot
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   * @return Closest point
   */
  Vec3 closestPoint(const Vec3 &position,
                    const Vec3 &wp_start,
                    const Vec3 &wp_end);

  /**
   * Calculate waypoint point
   *
   * @param position Actual position of robot
   * @param r Lookahead distance in meters
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   */
  Vec3 calculateWaypoint(const Vec3 &position,
                         const double r,
                         const Vec3 &wp_start,
                         const Vec3 &wp_end);

  /**
   * Calcuate which side the point is compared to waypoint track
   *
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   * @param position Position
   * @return
   *    - 0: Position is colinear with waypoint track
   *    - 1: Position is left of waypoint track
   *    - -1: Position is right of waypoint track
   */
  int pointLineSide(const Vec3 &wp_start,
                    const Vec3 &wp_end,
                    const Vec3 &position);

  /**
   * Calcuate cross track error
   *
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   * @param position Position
   * @return Cross track error
   */
  double crossTrackError(const Vec3 &wp_start,
                         const Vec3 &wp_end,
                         const Vec3 &position,
                         int mode = CTRACK_HORIZ);

  /**
   * Calculate waypoint tangent unit vector
   *
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   * @return Unit tagent vector
   */
  Vec3 waypointTangentUnitVector(const Vec3 &wp_start, const Vec3 &wp_end);

  /**
   * Calculate waypoint yaw
   *
   * @param wp_start Waypoint start
   * @param wp_end Waypoint end
   * @return Waypoint yaw
   */
  double waypointHeading(const Vec3 &wp_start, const Vec3 &wp_end);

  /**
   * Check whether waypoint is reached
   *
   * @param position Actual position of robot
   * @param waypoint Waypoint to check against
   * @param threshold Threshold in meters
   * @return
   *    - 0: Waypoint not reached
   *    - 1: Waypoint reached
   *    - -1: Not configured
   */
  int waypointReached(const Vec3 &position,
                      const Vec3 &waypoint,
                      const double threshold);
};

}  // namespace atl

#endif
