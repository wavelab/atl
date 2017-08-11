#ifndef ATL_CONTROL_POSITION_CONTROLLER_HPP
#define ATL_CONTROL_POSITION_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Position Controller
 */
class PositionController {
public:
  bool configured;

  double dt;
  PID x_controller;
  PID y_controller;
  PID z_controller;

  double roll_limit[2];
  double pitch_limit[2];
  double hover_throttle;

  Vec3 setpoints;
  Vec4 outputs;
  AttitudeCommand att_cmd;

  PositionController()
      : configured{false},
        dt{0.0},
        roll_limit{0.0, 0.0},
        pitch_limit{0.0, 0.0},
        hover_throttle{0.0},
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
   * @param setpoints Position setpoints
   * @param robot Actual robot pose
   * @param yaw Actual robot yaw
   * @param dt Time difference in seconds
   * @return controller output
   */
  Vec4 calculate(Vec3 setpoints, Pose robot_pose, double yaw, double dt);

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
