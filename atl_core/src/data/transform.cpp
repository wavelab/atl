#include "atl/data/transform.hpp"

namespace atl {

Mat3 rotx(const double angle) {
  Mat3 R;
  // clang-format off
  R << 1.0, 0.0, 0.0,
       0.0, cos(angle),
       -sin(angle), 0.0, sin(angle), cos(angle);
  // clang-format on
  return R;
}

Mat3 roty(const double angle) {
  Mat3 R;
  // clang-format off
  R << cos(angle), 0.0, -sin(angle),
       0.0, 1.0, 0.0,
       sin(angle), 0.0, cos(angle);
  // clang-format on
  return R;
}

Mat3 rotz(const double angle) {
  Mat3 R;
  // clang-format off
  R << cos(angle), -sin(angle), 0.0,
       sin(angle), cos(angle), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on
  return R;
}

Quaternion euler123ToQuat(const Vec3 &euler) {
  const double alpha = euler(0);
  const double beta = euler(1);
  const double gamma = euler(2);

  const double c1 = cos(alpha / 2.0);
  const double c2 = cos(beta / 2.0);
  const double c3 = cos(gamma / 2.0);
  const double s1 = sin(alpha / 2.0);
  const double s2 = sin(beta / 2.0);
  const double s3 = sin(gamma / 2.0);

  // euler 1-2-3 to quaternion
  const double w = c1 * c2 * c3 - s1 * s2 * s3;
  const double x = s1 * c2 * c3 + c1 * s2 * s3;
  const double y = c1 * s2 * c3 - s1 * c2 * s3;
  const double z = c1 * c2 * s3 + s1 * s2 * c3;

  return Quaternion{w, x, y, z};
}

Quaternion euler321ToQuat(const Vec3 &euler) {
  const double alpha = euler(0);
  const double beta = euler(1);
  const double gamma = euler(2);

  const double c1 = cos(alpha / 2.0);
  const double c2 = cos(beta / 2.0);
  const double c3 = cos(gamma / 2.0);
  const double s1 = sin(alpha / 2.0);
  const double s2 = sin(beta / 2.0);
  const double s3 = sin(gamma / 2.0);

  // euler 3-2-1 to quaternion
  const double w = c1 * c2 * c3 + s1 * s2 * s3;
  const double x = s1 * c2 * c3 - c1 * s2 * s3;
  const double y = c1 * s2 * c3 + s1 * c2 * s3;
  const double z = c1 * c2 * s3 - s1 * s2 * c3;

  return Quaternion{w, x, y, z};
}

Mat3 euler321ToRot(const Vec3 &euler) {
  const double phi = euler(0);
  const double theta = euler(1);
  const double psi = euler(2);

  // euler 3-2-1
  const double R11 = cos(theta) * cos(psi);
  const double R12 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
  const double R13 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);

  const double R21 = cos(theta) * sin(psi);
  const double R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
  const double R23 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);

  const double R31 = -sin(theta);
  const double R32 = sin(phi) * cos(theta);
  const double R33 = cos(phi) * cos(theta);

  // clang-format off
  Mat3 R;
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

Mat3 euler123ToRot(const Vec3 &euler) {
  const double phi = euler(0);
  const double theta = euler(1);
  const double psi = euler(2);

  // euler 1-2-3
  const double R11 = cos(theta) * cos(psi);
  const double R12 = cos(theta) * sin(psi);
  const double R13 = -sin(theta);

  const double R21 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
  const double R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
  const double R23 = sin(phi) * cos(theta);

  const double R31 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
  const double R32 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
  const double R33 = cos(phi) * cos(theta);

  // clang-format off
  Mat3 R;
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on

  return R;
}

Vec3 quatToEuler123(const Quaternion &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  const double qw2 = pow(qw, 2);
  const double qx2 = pow(qx, 2);
  const double qy2 = pow(qy, 2);
  const double qz2 = pow(qz, 2);

  const double phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2));
  const double theta = asin(2 * (qx * qz + qy * qw));
  const double psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2));

  return Vec3{phi, theta, psi};
}

Vec3 quatToEuler321(const Quaternion &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  const double qw2 = pow(qw, 2);
  const double qx2 = pow(qx, 2);
  const double qy2 = pow(qy, 2);
  const double qz2 = pow(qz, 2);

  const double phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const double theta = asin(2 * (qy * qw - qx * qz));
  const double psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  return Vec3{phi, theta, psi};
}

Mat3 quat2rot(const Quaternion &q) {
  const double qw = q.w();
  const double qx = q.x();
  const double qy = q.y();
  const double qz = q.z();

  // double qw2 = pow(qw, 2);
  const double qx2 = pow(qx, 2);
  const double qy2 = pow(qy, 2);
  const double qz2 = pow(qz, 2);

  // inhomogeneous form
  const double R11 = 1 - 2 * qy2 - 2 * qz2;
  const double R12 = 2 * qx * qy + 2 * qz * qw;
  const double R13 = 2 * qx * qz - 2 * qy * qw;

  const double R21 = 2 * qx * qy - 2 * qz * qw;
  const double R22 = 1 - 2 * qx2 - 2 * qz2;
  const double R23 = 2 * qy * qz + 2 * qx * qw;

  const double R31 = 2 * qx * qz + 2 * qy * qw;
  const double R32 = 2 * qy * qz - 2 * qx * qw;
  const double R33 = 1 - 2 * qx2 - 2 * qy2;

  // // homogeneous form
  // R11 = qx2 + qx2 - qy2 - qz2;
  // R12 = 2 * (qx * qy - qw * qz);
  // R13 = 2 * (qw * qy + qx * qz);
  //
  // R21 = 2 * (qx * qy + qw * qz);
  // R22 = qw2 - qx2 + qy2 - qz2;
  // R23 = 2 * (qy * qz - qw * qx);
  //
  // R31 = 2 * (qx * qz - qw * qy);
  // R32 = 2 * (qw * qx + qy * qz);
  // R33 = qw2 - qx2 - qy2 + qz2;

  // clang-format off
  Mat3 R;
  R << R11, R12, R13,
       R21, R22, R23,
       R31, R32, R33;
  // clang-format on
  return R;
}

Vec3 enu2nwu(const Vec3 &enu) {
  // ENU frame:  (x - right, y - forward, z - up)
  // NWU frame:  (x - forward, y - left, z - up)
  return Vec3{enu(1), -enu(0), enu(2)};
}

Vec3 edn2nwu(const Vec3 &edn) {
  // camera frame:  (x - right, y - down, z - forward)
  // NWU frame:  (x - forward, y - left, z - up)
  return Vec3{edn(2), -edn(0), -edn(1)};
}

Vec3 edn2enu(const Vec3 &edn) {
  // camera frame:  (x - right, y - down, z - forward)
  // ENU frame:  (x - right, y - forward, z - up)
  return Vec3{edn(0), edn(2), -edn(1)};
}

Vec3 nwu2enu(const Vec3 &nwu) {
  // NWU frame:  (x - forward, y - left, z - up)
  // ENU frame:  (x - right, y - forward, z - up)
  return Vec3{-nwu(1), nwu(0), nwu(2)};
}

Vec3 nwu2ned(const Vec3 &nwu) {
  // NWU frame:  (x - forward, y - left, z - up)
  // NED frame:  (x - forward, y - right, z - down)
  return Vec3{nwu(0), -nwu(1), -nwu(2)};
}

Vec3 ned2enu(const Vec3 &ned) {
  // NED frame:  (x - forward, y - right, z - down)
  // ENU frame:  (x - right, y - forward, z - up)
  return Vec3{ned(1), ned(0), -ned(2)};
}

Vec3 ned2nwu(const Vec3 &ned) {
  // NED frame:  (x - forward, y - right, z - down)
  // NWU frame:  (x - forward, y - left, z - up)
  return Vec3{ned(0), -ned(1), -ned(2)};
}

Quaternion nwu2ned(const Quaternion &nwu) {
  return Quaternion{nwu.w(), nwu.x(), -nwu.y(), -nwu.z()};
}

Quaternion ned2nwu(const Quaternion &ned) {
  return Quaternion{ned.w(), ned.x(), -ned.y(), -ned.z()};
}

Quaternion enu2nwu(const Quaternion &enu) {
  return Quaternion{enu.w(), enu.y(), -enu.x(), enu.z()};
}

} // namespace atl
