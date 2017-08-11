#ifndef ATL_DATA_LANDING_TARGET_HPP
#define ATL_DATA_LANDING_TARGET_HPP

#include "atl/utils/utils.hpp"

namespace atl {

struct LandingTarget {
  Vec3 position_bf{0.0, 0.0, 0.0};
  Vec3 velocity_bf{0.0, 0.0, 0.0};
  bool detected = false;
  bool losted = false;
  struct timespec first_seen = {0, 0};
  struct timespec last_seen = {0, 0};

  double lost_threshold = 1000.0;

  LandingTarget() {}
  bool isTargetLosted();
  void setTargetPosition(Vec3 position);
  void setTargetVelocity(Vec3 velocity);
  double tracked();
  void reset();
  void update(bool detected);
};

} // namespace atl
#endif
