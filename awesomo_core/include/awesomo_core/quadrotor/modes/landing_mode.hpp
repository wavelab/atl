#ifndef __AWESOMO_QUADROTOR_MODES_LANDING_MODE_HPP__
#define __AWESOMO_QUADROTOR_MODES_LANDING_MODE_HPP__

#include "awesomo_core/quadrotor/modes/base_mode.hpp"


namespace awesomo {

class LandingMode : public BaseMode {
public:
  // height update
  float period;
  float descend_multiplier;
  float recover_multiplier;

  // disarm conditions
  float belief_threshold;
  Vec3 cutoff_position;

  LandingMode(void);
  int configure(std::string config_file);
};


}  // end of awesomo namespace
#endif
