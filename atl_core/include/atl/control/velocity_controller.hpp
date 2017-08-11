#ifndef ATL_CONTROL_VELOCITY_CONTROLLER_HPP
#define ATL_CONTROL_VELOCITY_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Velocity Controller
 */
class VelocityController {
public:
  bool configured;

  double dt;
  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  double roll_limit[2];
  double pitch_limit[2];
  double throttle_limit[2];

  Vec3 setpoints;
  Vec4 outputs;
  AttitudeCommand att_cmd;

  VelocityController()
      : configured{false},
        dt{0.0},
        roll_limit{0.0, 0.0},
        pitch_limit{0.0, 0.0},
        throttle_limit{0.0, 0.0},
        setpoints{Vec3::Zero()},
        outputs{Vec4::Zero()} {}

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
   * @return controller output
   */
  Vec4 calculate(Vec3 setpoints, Vec3 actual, double dt);

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
