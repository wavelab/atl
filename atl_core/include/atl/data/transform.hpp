#ifndef ATL_DATA_TRANSFORM_HPP
#define ATL_DATA_TRANSFORM_HPP

#include "atl/utils/utils.hpp"
#include "atl/data/pose.hpp"
#include "atl/data/position.hpp"

namespace atl {

class Transform {
public:
  std::string into;
  std::string from;
  Mat4 data;

  Transform(const std::string &into, const std::string &from, const Mat4 &mat)
      : into{into}, from{from}, data(mat) {}

  Transform(const std::string &into,
            const std::string &from,
            const Mat3 &R,
            const Vec3 &t = Vec3::Zero())
      : into{into}, from{from} {
    // rotation component
    for (int i = 0; i < 3; i++) {
      this->data(i, 0) = R(i, 0);
      this->data(i, 1) = R(i, 1);
      this->data(i, 2) = R(i, 2);
    }

    // translation component
    this->data(0, 3) = t(0);
    this->data(1, 3) = t(1);
    this->data(2, 3) = t(2);

    // homogenous component
    this->data(3, 0) = 0.0;
    this->data(3, 1) = 0.0;
    this->data(3, 2) = 0.0;
    this->data(3, 3) = 1.0;
  }

  /// Transform pose
  friend Pose operator*(const Transform &T, const Pose &pose) {
    assert(T.from == pose.frame);
    const Mat4 p_transformed = T.data * pose.transformMatrix();
    const Mat3 R_transformed = p_transformed.block(0, 0, 3, 3);
    const Vec3 t_transformed = p_transformed.block(0, 3, 3, 1);
    return Pose{T.into, t_transformed, R_transformed};
  }

  /// Transform position
  friend Position operator*(const Transform &T, const Position &x) {
    assert(T.from == x.frame);
    return Position{T.into, (T.data * x.homogeneous()).block(0, 0, 3, 1)};
  }

  /// Transform vector
  friend Vec3 operator*(const Transform &T, const Vec3 &x) {
    const Vec4 x_homo{x(0), x(1), x(2), 1.0};
    return Vec3{(T.data * x_homo).block(0, 0, 3, 1)};
  }

  /// Transform vector
  friend Vec4 operator*(const Transform &T, const Vec4 &x) {
    return T.data * x;
  }

  /// Chain transforms
  friend Transform operator*(const Transform &T1, const Transform &T2) {
    const Mat4 T = T1.data * T2.data;
    return Transform{T1.into, T2.from, T};
  }

  /// Transform quaternion
  friend Quaternion operator*(const Transform &T, const Quaternion &x) {
    const Vec4 x_homo{x.x(), x.y(), x.z(), 1.0};
    const Vec3 result = (T.data * x_homo).block(0, 0, 3, 1);
    return Quaternion{x.w(), result(0), result(1), result(2)};
  }
};

/**
 * Transform from camera to world frame
 *
 * Assumes:
 * - camera frame is EDN
 * - world frame is NWU
 */
// clang-format off
const Transform T_W_C{"W", "C", (Mat4() << 0.0, 0.0, 1.0, 0.0,
                                           -1.0, 0.0, 0.0, 0.0,
                                           0.0, -1.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Obtain yaw only from quaternion
 */
static inline Quaternion yaw(const Quaternion &q) {
  Quaternion q_yaw{q.w(), 0.0, 0.0, q.z()};
  return q_yaw.normalized();
}

/**
 * Obtain yaw only from euler angles
 */
static inline Vec3 yaw(const Vec3 &rpy) { return Vec3{0.0, 0.0, rpy(2)}; }

/**
 * Transform from world to body frame
 */
class T_B_W : public Transform {
public:
  T_B_W(const Quaternion &q_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"B", "W", q_W.toRotationMatrix().inverse(), -1.0 * t_W} {}
  T_B_W(const Vec3 &rpy_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"B", "W", euler123ToRot(rpy_W), -1.0 * t_W} {}
  T_B_W(const Mat3 &R_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"B", "W", R_W, -1.0 * t_W} {}
};

/**
 * Transform from inertial to body planar frame
 */
class T_P_W : public Transform {
public:
  T_P_W(const Quaternion &q_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"P", "W", yaw(q_W).toRotationMatrix().inverse(), -1.0 * t_W} {
  }
  T_P_W(const Vec3 &rpy_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"P", "W", euler123ToRot(yaw(rpy_W)), -1.0 * t_W} {}
  T_P_W(const Mat3 &R_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"P", "W", R_W, -1.0 * t_W} {}
};

/**
 * Transform from body to inertial frame
 */
class T_W_B : public Transform {
public:
  T_W_B(const Quaternion &q_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"W", "B", q_W.toRotationMatrix(), t_W} {}
  T_W_B(const Vec3 &rpy_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"W", "B", euler321ToRot(rpy_W), t_W} {}
  T_W_B(const Mat3 &R_W, const Vec3 &t_W = Vec3::Zero())
      : Transform{"W", "B", R_W, t_W} {}
};

} // namespace atl
#endif
