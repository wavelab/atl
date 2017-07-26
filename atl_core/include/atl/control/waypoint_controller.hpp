#ifndef ATL_CONTROL_WAYPOINT_CONTROLLER_HPP
#define ATL_CONTROL_WAYPOINT_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"
#include "atl/control/pid.hpp"

namespace atl {

/**
 * Waypoint Controller
 */
class WaypointController {
public:
  bool configured = false;

  double dt = 0.0;

  double vx_error_prev = 0.0;
  double vy_error_prev = 0.0;
  double vz_error_prev = 0.0;

  double vx_k_p = 0.0;
  double vx_k_i = 0.0;
  double vx_k_d = 0.0;

  double vy_k_p = 0.0;
  double vy_k_i = 0.0;
  double vy_k_d = 0.0;

  double vz_k_p = 0.0;
  double vz_k_i = 0.0;
  double vz_k_d = 0.0;

  double yaw_k_p = 0.0;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double throttle_limit[2] = {0.0, 0.0};

  Vec3 pos_setpoints = Vec3::Zero();
  Vec3 vel_setpoints = Vec3::Zero();
  Vec4 outputs = Vec4::Zero();
  AttitudeCommand att_cmd;

  WaypointController() {}

  /**
   * Configure
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Calculate controller outputs
   *
   * @param pos_setpoints Position setpoints
   * @param vel_setpoints Velocity setpoints
   * @param yaw_setpoint Yaw setpoint
   * @param pose Actual robot pose
   * @param vel Actual robot velocity
   * @param dt Time difference in seconds
   * @return controller output
   */
  Vec4 calculate(const Vec3 &pos_setpoints,
                 const Vec3 &vel_setpoints,
                 const double yaw_setpoint,
                 const Pose &pose,
                 const Vec3 &vel,
                 const double dt);

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
