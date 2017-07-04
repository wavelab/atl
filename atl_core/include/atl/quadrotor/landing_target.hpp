#ifndef ATL_QUADROTOR_LANDING_TARGET_HPP
#define ATL_QUADROTOR_LANDING_TARGET_HPP

#include "atl/utils/utils.hpp"

namespace atl {

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

}  // namespace atl
#endif
