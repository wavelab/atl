#ifndef ATL_CONTROL_TRACKING_CONTROLLER_HPP
#define ATL_CONTROL_TRACKING_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Tracking Controller
 */
class TrackingController {
public:
  bool configured = false;

  double dt = 0.0;
  PID x_controller;
  PID y_controller;
  PID z_controller;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double hover_throttle = 0.0;
  Vec3 track_offset{0.0, 0.0, 0.0};

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};
  AttitudeCommand att_cmd;

  TrackingController() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Calculate controller outputs
   *
   * @param pos_errors Tracking errors in body frame
   * @param yaw Actual robot yaw
   * @param dt Time difference in seconds
   *
   * @return controller output
   */
  AttitudeCommand calculate(
    const Vec3 &pos_errors, const double yaw, const double dt);

  /**
   * Calculate controller outputs
   *
   * @param target_pos_bf Target position in body frame
   * @param pos Robot position in inertial frame
   * @param pos_prev Previous robot position in inertial frame
   * @param yaw Robot yaw
   * @param dt Time difference in seconds
   *
   * @return controller output
   */
  AttitudeCommand calculate(
    const Vec3 &target_pos_bf,
    const Vec3 &pos,
    const Vec3 &pos_prev,
    const double yaw,
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
