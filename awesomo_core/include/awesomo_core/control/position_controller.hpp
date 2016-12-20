#ifndef __AWESOMO_CONTROL_POSITION_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_POSITION_CONTROLLER_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

class PositionController {
public:
  PID x_controller;
  PID y_controller;
  PID z_controller;

  double roll_limit[2];
  double pitch_limit[2];

  double setpoint_roll;
  double setpoint_pitch;
  double setpoint_throttle;

  double output_roll;
  double output_pitch;
  double output_throttle;

  PositionController(void);
  void loadConfig(const std::string config_file);
  VecX calculate(VecX setpoints, VecX actual, double yaw, double dt);
  void reset(void);
};

}  // end of awesomo namespace
#endif
