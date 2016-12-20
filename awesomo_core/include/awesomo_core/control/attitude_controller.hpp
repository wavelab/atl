#ifndef __AWESOMO_CONTROL_ATTITUDE_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_ATTITUDE_CONTROLLER_HPP__

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

class AttitudeController {
public:
  bool configured;

  PID roll_controller;
  PID pitch_controller;
  PID yaw_controller;

  double setpoints[4];
  double outputs[4];

  AttitudeController(void);
  VecX calculate(Vec4 setpoints, Vec4 actual, double dt);
  VecX calculate(Vec4 psetpoints, Vec4 vsetpoints, Vec4 actual, double dt);
};

}  // end of awesomo namespace
#endif
