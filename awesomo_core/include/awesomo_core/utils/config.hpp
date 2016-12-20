#ifndef __AWESOMO_UTILS_CONFIG_HPP__
#define __AWESOMO_UTILS_CONFIG_HPP__

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

class LandingConfig {
public:
  // height update
  float period;
  float descend_multiplier;
  float recover_multiplier;

  // disarm conditions
  float belief_threshold;
  Vec3 cutoff_position;

  // constructor
  LandingConfig(void);
  LandingConfig(float period,
                float desend_multiplier,
                float recover_multiplier,
                float belief_threshold,
                Vec3 cutoff_position);
};

}  // end of awesomo namespace
#endif
