#include "awesomo_core/control/attitude_controller.hpp"


namespace awesomo {

AttitudeController::AttitudeController(void) {
  this->configured = false;

  this->roll_controller = PID(0.0, 0.0, 0.0);
  this->pitch_controller = PID(0.0, 0.0, 0.0);
  this->yaw_controller = PID(0.0, 0.0, 0.0);

  this->setpoints[0] = 0;
  this->setpoints[1] = 0;
  this->setpoints[2] = 0;
  this->setpoints[3] = 0;

  this->outputs[0] = 0;
  this->outputs[1] = 0;
  this->outputs[2] = 0;
  this->outputs[3] = 0;
}

VecX AttitudeController::calculate(Vec4 setpoints, Vec4 actual, double dt) {
  VecX outputs(4);
  double r, p, y, t;
  double max_thrust;

  // roll pitch yaw
  r = this->roll_controller.calculate(setpoints(0), actual(0), dt);
  p = this->pitch_controller.calculate(setpoints(1), actual(1), dt);
  y = this->yaw_controller.calculate(setpoints(2), actual(2), dt);

  // thrust
  max_thrust = 5.0;
  t = max_thrust * setpoints(3);          // convert relative thrust to true thrust
  t = (t > max_thrust) ? max_thrust : t;  // limit thrust
  t = (t < 0) ? 0.0 : t;                  // limit thrust

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

  // keep track of setpoints and outputs
  for (int i = 0; i < 4; i++) {
    this->setpoints[i] = setpoints(i);
    this->outputs[i] = outputs(i);
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
