#include "awesomo_core/quadrotor/modes/landing_mode.hpp"


namespace awesomo {

LandingMode::LandingMode(void) {
  this->period = 0;
  this->descend_multiplier = 0;
  this->recover_multiplier = 0;
  this->cutoff_position << 0, 0, 0;
  this->belief_threshold = 0;
}

int LandingMode::configure(std::string config_file) {
  return 0;
}

}  // end of awesomo namespace
