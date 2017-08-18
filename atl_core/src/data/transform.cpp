#include "atl/data/transform.hpp"

namespace atl {

Vec3 operator*(const Transform &T, const Vec3 &x) {
  const Vec4 x_homo{x(0), x(1), x(2), 1.0};
  return (((Mat4) T) * x_homo).block(0, 0, 3, 1);
}

} // namespace atl
