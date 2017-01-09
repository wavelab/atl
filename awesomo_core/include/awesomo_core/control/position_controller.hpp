#ifndef __AWESOMO_CONTROL_POSITION_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_POSITION_CONTROLLER_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

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

  PositionController(void);
  int configure(std::string config_file);
  Vec4 calculate(Vec3 setpoints, Vec4 actual, double yaw, double dt);
  void reset(void);
  void printOutputs(void);
  void printErrors(void);
};

}  // end of awesomo namespace
#endif
