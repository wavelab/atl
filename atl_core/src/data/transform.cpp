#include "atl/data/transform.hpp"

namespace atl {

Vec3 Transform::operator*(const Vec3 &x) {
  Mat3 R = this->orientation.toRotationMatrix();
  Vec3 t = this->translation;

  Mat34 T;
  T << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1),
      R(2, 0), R(2, 1), R(2, 2), t(2);

  return T * x;
}

} // namespace atl
