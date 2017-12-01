#ifndef ATL_CONTROL_POSITION_CONTROLLER_HPP
#define ATL_CONTROL_POSITION_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/data/data.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Position Controller
 */
class PositionController {
public:
  bool configured = false;

  double dt = 0.0;
  PID x_controller;
  PID y_controller;
  PID z_controller;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double hover_throttle = 0.0;

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  PositionController() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Update controller
   *
   * @param setpoints Position setpoints
   * @param pose Actual pose
   * @param yaw Actual yaw
   * @param dt Time difference in seconds
   *
   * @return
   *    Attitude command as a vector of size 4:
   *    (roll, pitch, yaw, throttle)
   */
  Vec4 update(const Vec3 &setpoints,
              const Pose &pose,
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

} // namespace atl
#endif
