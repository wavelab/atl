#include "atl/data/transform.hpp"

namespace atl {

Vec3 operator*(const Transform &T, const Vec3 &x) {
  const Vec4 x_homo{x(0), x(1), x(2), 1.0};
  return (((Mat4) T) * x_homo).block(0, 0, 3, 1);
}

Quaternion operator*(const Transform &T, const Quaternion &x) {
  const Vec4 x_homo{x.x(), x.y(), x.z(), 1.0};
  const Vec3 result = (((Mat4) T) * x_homo).block(0, 0, 3, 1);
  return Quaternion{x.w(), result(0), result(1), result(2)};
}

} // namespace atl
