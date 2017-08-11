#ifndef ATL_DATA_TWIST_HPP
#define ATL_DATA_TWIST_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Twist stores linear and angular velocity
 */
class Twist {
public:
  Vec3 linear{0.0, 0.0, 0.0};
  Vec3 angular{0.0, 0.0, 0.0};

  Twist() {}
};

} // namespace atl
#endif
