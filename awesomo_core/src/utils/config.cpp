#include "awesomo_core/utils/config.hpp"


namespace awesomo {

LandingConfig::LandingConfig(void) {
  this->period = 0;
  this->descend_multiplier = 0;
  this->recover_multiplier = 0;
  this->cutoff_position << 0, 0, 0;
  this->belief_threshold = 0;
}

LandingConfig::LandingConfig(float period,
                             float desend_multiplier,
                             float recover_multiplier,
                             float belief_threshold,
                             Eigen::Vector3d cutoff_position) {
  this->period = period;
  this->descend_multiplier = descend_multiplier;
  this->recover_multiplier = recover_multiplier;
  this->cutoff_position = cutoff_position;
  this->belief_threshold = belief_threshold;
}

}  // end of awesomo namespace
