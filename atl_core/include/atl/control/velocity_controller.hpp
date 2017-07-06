#ifndef ATL_CONTROL_VELOCITY_CONTROLLER_HPP
#define ATL_CONTROL_VELOCITY_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid_controller.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

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

  VelocityController(void);
  int configure(std::string config_file);
  Vec4 calculate(Vec3 setpoints, Vec3 actual, double dt);
  void reset(void);
  void printOutputs(void);
  void printErrors(void);
};

}  // namespace atl
#endif
