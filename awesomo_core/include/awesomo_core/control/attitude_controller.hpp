#ifndef __AWESOMO_CONTROL_ATTITUDE_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_ATTITUDE_CONTROLLER_HPP__

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

class AttitudeController {
public:
  PID roll_controller;
  PID pitch_controller;
  PID yaw_controller;

  AttitudeController(void);
  VecX calculate(Vec4 setpoints, Vec4 actual, double dt);
  VecX calculate(Vec4 psetpoints, Vec4 vsetpoints, Vec4 actual, double dt);
};

}  // end of awesomo namespace
#endif
