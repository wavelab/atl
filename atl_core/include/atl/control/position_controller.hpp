#ifndef ATL_CONTROL_POSITION_CONTROLLER_HPP
#define ATL_CONTROL_POSITION_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid_controller.hpp"
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

  PositionController();
  int configure(std::string config_file);
  Vec4 calculate(Vec3 setpoints, Pose robot_pose, double yaw, double dt);
  void reset();
  void printOutputs();
  void printErrors();
};

}  // namespace atl
#endif
