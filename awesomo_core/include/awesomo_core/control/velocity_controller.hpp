#ifndef __AWESOMO_CONTROL_VELOCITY_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_VELOCITY_CONTROLLER_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

class VelocityController {
public:
  bool configured;

  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  double hover_throttle;

  double roll_limit[2];
  double pitch_limit[2];

  double setpoints[3];
  double outputs[4];

  VelocityController(void);
  int configure(std::string config_file);
  VecX calculate(Vec3 setpoints, Vec3 actual, double yaw, double dt);
  void reset(void);
};

}  // end of awesomo namespace
#endif
