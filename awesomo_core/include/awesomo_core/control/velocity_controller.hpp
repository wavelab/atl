#ifndef __AWESOMO_CONTROL_VELOCITY_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_VELOCITY_CONTROLLER_HPP__

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {
namespace control {

class VelocityController {
public:
  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  VelocityController(void);
  VecX calculate(Vec3 setpoints, Vec3 actual, double yaw, double dt);
};

}  // end of control namespace
}  // end of awesomo namespace
#endif
