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

  double hover_throttle;

  double roll_limit[2];
  double pitch_limit[2];

  double setpoint_x;
  double setpoint_y;
  double setpoint_z;

  double output_roll;
  double output_pitch;
  double output_throttle;

  AttitudeCommand att_cmd;

  PositionController(void);
  int configure(std::string config_file);
  VecX calculate(VecX setpoints, VecX actual, double yaw, double dt);
  void reset(void);
  void printErrors(void);
  void printInputs(void);
  void printOutputs(void);
};

}  // end of awesomo namespace
#endif
