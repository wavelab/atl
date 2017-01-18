#ifndef __AWESOMO_QUADROTOR_LANDING_TARGET_HPP__
#define __AWESOMO_QUADROTOR_LANDING_TARGET_HPP__

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

class LandingTarget {
public:
  Vec3 position_bf;
  Vec3 velocity_bf;
  bool detected;
  bool losted;
  struct timespec first_seen;
  struct timespec last_seen;

  double lost_threshold;

  LandingTarget(void);
  bool isTargetLosted(void);
  void setTargetPosition(Vec3 position);
  void setTargetVelocity(Vec3 velocity);
  double tracked(void);
  void reset(void);
  void update(bool detected);
};

}  // end of awesomo namespace
#endif
