#include "awesomo_control/attitude_controller.hpp"


namespace awesomo {

AttitudeController::AttitudeController(void) {
  this->roll_controller = PID(12.0, 0.05, 1.15);
  this->pitch_controller = PID(12.0, 0.05, 1.15);
  this->yaw_controller = PID(1.0, 0.0, 0.5);
}

VecX AttitudeController::calculate(Vec4 setpoints, Vec4 actual, double dt) {
  double r, p, y, t;
  VecX outputs(4);
  double max_thrust;

  // roll pitch yaw
  r = this->roll_controller.calculate(setpoints(0), actual(0), dt);
  p = this->pitch_controller.calculate(setpoints(1), actual(1), dt);
  y = this->yaw_controller.calculate(setpoints(2), actual(2), dt);

  // thrust
  max_thrust = 5.0;
  t = max_thrust * setpoints(3);  // convert relative thrust to true thrust
  t /= fabs(cos(actual(0)) * cos(actual(1)));  // adjust for roll and pitch
  t = (t > max_thrust) ? max_thrust : t;       // limit thrust
  t = (t < 0) ? 0.0 : t;                       // limit thrust

  // map roll, pitch, yaw and thrust to motor outputs
  outputs(0) = -p - y + t;
  outputs(1) = -r + y + t;
  outputs(2) = p - y + t;
  outputs(3) = r + y + t;

  // limit outputs
  for (int i = 0; i < 4; i++) {
    if (outputs(i) > max_thrust) {
      outputs(i) = max_thrust;
    } else if (outputs(i) < 0.0) {
      outputs(i) = 0.0;
    }
  }

  return outputs;
}

VecX AttitudeController::calculate(Vec4 psetpoints,
                                   Vec4 vsetpoints,
                                   Vec4 actual,
                                   double dt) {
  Vec4 setpoints(4);
  setpoints = psetpoints + vsetpoints;
  return this->calculate(setpoints, actual, dt);
}

}  // end of awesomo namespace
