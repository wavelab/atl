#ifndef __atl_CONTROL_VELOCITY_CONTROLLER_HPP__
#define __atl_CONTROL_VELOCITY_CONTROLLER_HPP__

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl_core/utils/utils.hpp"
#include "atl_core/control/pid_controller.hpp"


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

}  // end of atl namespace
#endif
