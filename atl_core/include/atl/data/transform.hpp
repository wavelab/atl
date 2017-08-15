#ifndef ATL_DATA_TRANSFORM_HPP
#define ATL_DATA_TRANSFORM_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Transform
 */
struct Transform {
  Vec3 translation{0.0, 0.0, 0.0};
  Quaternion rotation{1.0, 0.0, 0.0, 0.0};

  Transform(const Vec3 &t, const Quaternion &rpy)
      : translation{t}, rotation{rpy} {}

  Transform(const Vec3 &t, const Vec3 &rpy) : translation{t} {
    euler2quat(rpy, 321, this->rotation);
  }

  Vec3 operator*(const Vec3 &x);
};

} // namespace atl
#endif
