#ifndef ATL_CONTROL_WAYPOINT_CONTROLLER_HPP
#define ATL_CONTROL_WAYPOINT_CONTROLLER_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/data/data.hpp"
#include "atl/mission/mission.hpp"
#include "atl/utils/utils.hpp"

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

  PID at_controller;
  PID ct_controller;
  PID z_controller;
  PID yaw_controller;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double hover_throttle = 0.0;

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  bool blackbox_enable = true;
  double blackbox_rate = 1.0;
  double blackbox_dt = 0.0;
  std::ofstream blackbox;

  WaypointController() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return
   *    - 0: Success
   *    - -1: Failed to load config file
   *    - -2: Failed to load mission file
   */
  int configure(const std::string &config_file);

  /**
   * Prepare black box file
   *
   * @param blackbox_file Path to store black box file
   * @return
   *    - 0: Success
   *    - -1: Failed to store black box file
   */
  int prepBlackbox(const std::string &blackbox_file);

  /**
   * Record position and waypoint
   *
   * @param pos Position
   * @param waypoint Waypoint
   * @param dt Difference in time (seconds)
   *
   * @return
   *    - 0: Success
   *    - -1: Failure
   */
  int record(const Vec3 &pos, const Vec3 &waypoint);

  /**
   * Calculate yaw to waypoint based on position
   *
   * @param pos Position
   * @param waypoint Waypoint
   *
   * @return Yaw to waypoint
   */
  double calcYawToWaypoint(const Vec3 &waypoint, const Vec3 &position);

  /**
   * Update controller
   *
   * @param pose Actual pose
   * @param vel Actual velocity
   * @param dt Time difference in seconds
   * @param dt Time difference in seconds
   *
   * @return
   *   - 0: Success
   *   - -1: Not configured
   *   - -2: No more waypoints
   */
  int update(Mission &mission,
             const Pose &pose,
             const Vec3 &vel,
             const double dt);

  /**
   * Reset controller errors to 0
   */
  void reset();
};

} // namespace atl
#endif
