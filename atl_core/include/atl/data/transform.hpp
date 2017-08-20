#ifndef ATL_DATA_TRANSFORM_HPP
#define ATL_DATA_TRANSFORM_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Right hand rotation Matrix in x-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rotx(const double angle);

/**
 * Right hand rotation Matrix in y-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 roty(const double angle);

/**
 * Right hand rotation Matrix in z-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rotz(const double angle);

/**
 * Convert Euler 1-2-3 angles to quaternion
 * @param euler Input Euler angles
 * @return Output quaternion
 */
Quaternion euler123ToQuat(const Vec3 &euler);

/**
 * Convert Euler 3-2-1 angles to quaternion
 * @param euler Input Euler angles
 * @return Output quaternion
 */
Quaternion euler321ToQuat(const Vec3 &euler);

/**
 * Convert Euler 1-2-3 angles to rotation matrix
 * @param euler Input Euler angles
 * @return R Output rotation matrix
 */
Mat3 euler123ToRot(const Vec3 &euler);

/**
 * Convert Euler 3-2-1 angles to rotation matrix
 * @param euler Input Euler angles
 * @return R Output rotation matrix
 */
Mat3 euler321ToRot(const Vec3 &euler);

/**
 * Convert quanternion to Euler 1-2-3 angles
 * @param q Input quaternion
 * @return euler Output Euler angles
 */
Vec3 quatToEuler123(const Quaternion &q);

/**
 * Convert quanternion to Euler 3-2-1 angles
 * @param q Input quaternion
 * @return euler Output Euler angles
 */
Vec3 quatToEuler321(const Quaternion &q);

/**
 * Convert Quaternion to rotation matrix
 * @param q Input quaternion
 * @param R Output rotation matrix
 */
Mat3 quat2rot(const Quaternion &q);

/**
 * Convert from ENU to NWU
 * @param enu ENU vector
 * @return nwu NWU vector
 */
Vec3 enu2nwu(const Vec3 &enu);

/**
 * Convert from EDN to NWU
 * @param enu EDN vector
 * @return nwu NWU vector
 */
Vec3 edn2nwu(const Vec3 &edn);

/**
 * Convert from EDN to ENU
 * @param enu EDN vector
 * @return enu ENU vector
 */
Vec3 edn2enu(const Vec3 &edn);

/**
 * Convert from NWU to ENU
 * @param nwu NWU vector
 * @return enu ENU vector
 */
Vec3 nwu2enu(const Vec3 &nwu);

/**
 * Convert from NWU to NED
 * @param nwu NWU vector
 * @return enu NED vector
 */
Vec3 nwu2ned(const Vec3 &nwu);

/**
 * Convert from NED to ENU
 * @param ned NED vector
 * @return enu ENU vector
 */
Vec3 ned2enu(const Vec3 &ned);

/**
 * Convert from NED to NWU
 * @param ned NED vector
 * @return enu NWU vector
 */
Vec3 ned2nwu(const Vec3 &ned);

/**
 * Convert from NWU to NED
 * @param nwu NWU quaternion
 * @return ned NED quaternion
 */
Quaternion nwu2ned(const Quaternion &nwu);

/**
 * Conver from NED to NWU
 * @param ned NED quaternion
 * @param nwu NWU quaternion
 */
Quaternion ned2nwu(const Quaternion &ned);

/**
 * Conver from ENU to NWU
 * @param ned ENU quaternion
 * @param nwu NWU quaternion
 */
Quaternion enu2nwu(const Quaternion &enu);

class Transform {
public:
  Mat4 data;

  Transform() : data{Mat4::Identity()} {}
  Transform(const Mat4 &mat) : data(mat) {}
  Transform(const Mat3 &R, const Vec3 &t = Vec3::Zero()) {
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

  /**
  * Transform position or velocity vector of size 3
  */
  friend Vec3 operator*(const Transform &T, const Vec3 &x) {
    const Vec4 x_homo{x(0), x(1), x(2), 1.0};
    return (T.data * x_homo).block(0, 0, 3, 1);
  }

  /**
  * Chain transform
  */
  friend Mat4 operator*(const Transform &T1, const Transform &T2) {
    return T1.data * T2.data;
  }

  /**
  * Transform quaternion
  */
  friend Quaternion operator*(const Transform &T, const Quaternion &x) {
    const Vec4 x_homo{x.x(), x.y(), x.z(), 1.0};
    const Vec3 result = (T.data * x_homo).block(0, 0, 3, 1);
    return Quaternion{x.w(), result(0), result(1), result(2)};
  }
};

/**
 * Transform from NED to NWU
 */
// clang-format off
const Transform T_nwu_ned{(Mat4() << 1.0, 0.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, -1.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Transform from NWU to NED
 */
// clang-format off
const Transform T_ned_nwu{(Mat4() << 1.0, 0.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, -1.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Transform from EDN to NWU
 */
// clang-format off
const Transform T_nwu_edn{(Mat4() << 0.0, 0.0, 1.0, 0.0,
                                     -1.0, 0.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0).finished()};
// clang-format on

/**
 * Transform from NWU to EDN
 */
// clang-format off
const Transform T_edn_nwu{(Mat4() << 0.0, -1.0, 0.0, 0.0,
                                     0.0, 0.0, -1.0, 0.0,
                                     -1.0, 0.0, 0.0, 0.0,
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
 * Transform from inertial to body frame
 */
class T_bf_if : public Transform {
public:
  T_bf_if(const Quaternion &q_if, const Vec3 &pos_if = Vec3::Zero())
      : Transform{q_if.toRotationMatrix().inverse(), -1.0 * pos_if} {}
  T_bf_if(const Vec3 &rpy_if, const Vec3 &pos_if = Vec3::Zero())
      : Transform{euler123ToRot(rpy_if), -1.0 * pos_if} {}
  T_bf_if(const Mat3 &R, const Vec3 &pos_if = Vec3::Zero())
      : Transform{R, -1.0 * pos_if} {}
};

/**
 * Transform from inertial to body planar frame
 */
class T_bpf_if : public Transform {
public:
  T_bpf_if(const Quaternion &q_if, const Vec3 &pos_if = Vec3::Zero())
      : Transform{yaw(q_if).toRotationMatrix().inverse(), -1.0 * pos_if} {}
  T_bpf_if(const Vec3 &rpy_if, const Vec3 &pos_if = Vec3::Zero())
      : Transform{euler123ToRot(yaw(rpy_if)), -1.0 * pos_if} {}
  T_bpf_if(const Mat3 &R, const Vec3 &pos_if = Vec3::Zero())
      : Transform{R, -1.0 * pos_if} {}
};

/**
 * Transform from body to inertial frame
 */
class T_if_bf : public Transform {
public:
  T_if_bf(const Quaternion &q_if, const Vec3 &pos_if = Vec3::Zero())
      : Transform{q_if.toRotationMatrix(), pos_if} {}
  T_if_bf(const Vec3 &rpy_if, const Vec3 &pos_if = Vec3::Zero())
      : Transform{euler321ToRot(rpy_if), pos_if} {}
  T_if_bf(const Mat3 &R, const Vec3 &pos_if = Vec3::Zero())
      : Transform{R, pos_if} {}
};

} // namespace atl
#endif
