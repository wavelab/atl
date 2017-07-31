#include "atl/control/pid.hpp"

namespace atl {

double PID::update(const double setpoint,
                   const double input,
                   const double dt) {
  // calculate errors
  double error = setpoint - input;
  this->error_sum += error * dt;

  // calculate output
  this->error_p = this->k_p * error;
  this->error_i = this->k_i * this->error_sum;
  this->error_d = this->k_d * (error - this->error_prev) / dt;
  double output = this->error_p + this->error_i + this->error_d;

  // update error
  this->error_prev = error;

  return output;
}

double PID::update(const double error, const double dt) {
  return this->update(error, 0.0, dt);
}

void PID::reset() {
  this->error_prev = 0.0;
  this->error_sum = 0.0;

  this->error_p = 0.0;
  this->error_i = 0.0;
  this->error_d = 0.0;
}

}  // end of atl_control namespace
