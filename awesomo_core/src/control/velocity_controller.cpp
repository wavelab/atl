#include "awesomo_core/control/velocity_controller.hpp"


namespace awesomo {

VelocityController::VelocityController(void) {
  this->vx_controller = PID(0.9, 0.0, 0.03);
  this->vy_controller = PID(0.9, 0.0, 0.03);
  this->vz_controller = PID(1.0, 0.0, 0.0);
}

VecX VelocityController::calculate(Vec3 setpoints,
                                   Vec3 actual,
                                   double yaw,
                                   double dt) {
  VecX outputs(4);
  double r, p, y, t;

  r = -this->vy_controller.calculate(setpoints(1), actual(1), dt);
  p = this->vx_controller.calculate(setpoints(0), actual(0), dt);
  y = yaw;
  t = 0.5 + this->vz_controller.calculate(setpoints(2), actual(2), dt);
  outputs << r, p, y, t;

  // limit roll, pitch
  for (int i = 0; i < 2; i++) {
    if (outputs(i) > deg2rad(30.0)) {
      outputs(i) = deg2rad(30.0);
    } else if (outputs(i) < deg2rad(-30.0)) {
      outputs(i) = deg2rad(-30.0);
    }
  }

  // limit thrust
  if (outputs(3) > 1.0) {
    outputs(3) = 1.0;
  } else if (outputs(3) < 0.0) {
    outputs(3) = 0.0;
  }

  return outputs;
}

}  // end of awesomo namespace
