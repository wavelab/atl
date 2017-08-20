#ifndef ATL_CONTROL_VELOCITY_CONTROLLER_HPP
#define ATL_CONTROL_VELOCITY_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/data/data.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Velocity Controller
 */
class VelocityController {
public:
  bool configured = false;

  double dt = 0.0;
  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double throttle_limit[2] = {0.0, 0.0};

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  VelocityController() {}

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
   * @param setpoints Velocity setpoints in inertial frame
   * @param actual Actual velocity in inertial frame
   * @param dt Time difference in seconds
   *
   * @return
   *    Attitude command as a vector of size 4:
   *    (roll, pitch, yaw, throttle)
   */
  Vec4 calculate(const Vec3 &setpoints, const Vec3 &actual, const double dt);

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

} // namespace atl
#endif
