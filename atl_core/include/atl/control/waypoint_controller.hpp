#ifndef ATL_CONTROL_WAYPOINT_CONTROLLER_HPP
#define ATL_CONTROL_WAYPOINT_CONTROLLER_HPP

#include <iomanip>
#include <deque>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"
#include "atl/control/pid.hpp"

namespace atl {

#define CTRACK_HORIZ 0
#define CTRACK_3D 1

/**
 * Waypoint Controller
 */
class WaypointController {
public:
  bool configured = false;

  double dt = 0.0;

  double vel_desired = 0.5;

  double error_ct_sum = 0.0;
  double error_ct_prev = 0.0;
  double error_at_sum = 0.0;
  double error_at_prev = 0.0;
  double error_z_sum = 0.0;
  double error_z_prev = 0.0;

  double vx_k_p = 0.0;
  double vx_k_i = 0.0;
  double vx_k_d = 0.0;

  double y_k_p = 0.0;
  double y_k_i = 0.0;
  double y_k_d = 0.0;

  double z_k_p = 0.0;
  double z_k_i = 0.0;
  double z_k_d = 0.0;

  double yaw_k_p = 0.0;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double throttle_limit[2] = {0.0, 0.0};

  double look_ahead_dist = 0.5;
  double wp_threshold = 0.2;
  std::deque<Vec3> waypoints;
  Vec3 wp_start = Vec3::Zero();
  Vec3 wp_end = Vec3::Zero();

  Vec3 pos_setpoints = Vec3::Zero();
  Vec3 vel_setpoints = Vec3::Zero();
  Vec4 outputs = Vec4::Zero();
  AttitudeCommand att_cmd;

  WaypointController() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

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
  int waypointUpdate(const Vec3 &position, Vec3 &waypoint);


  /**
   * Update controller
   *
   * @param pose Actual pose
   * @param vel Actual velocity
   * @param dt Time difference in seconds
   * @param dt Time difference in seconds
   * @return
   *   - 0: Success
   *   - -1: Not configured
   *   - -2: No more waypoints
   */
  int update(const Pose &pose, const Vec3 &vel, const double dt, Vec4 &u);

  /**
   * Reset controller errors to 0
   */
  void reset();

  /**
   * Print controller outputs
   */
  void printOutputs();

  /**
   * Print controller errors
   */
  void printErrors();
};

}  // namespace atl
#endif
