#include "atl/utils/math.hpp"

namespace atl {

int randi(int ub, int lb) { return rand() % lb + ub; }

double randf(const double ub, const double lb) {
  const double f = (double) rand() / RAND_MAX;
  return lb + f * (ub - lb);
}

int sign(const double x) {
  if (fltcmp(x, 0.0) == 0) {
    return 0;
  } else if (x < 0) {
    return -1;
  }
  return 1;
}

int fltcmp(const double f1, const double f2) {
  if (fabs(f1 - f2) <= 0.0001) {
    return 0;
  } else if (f1 > f2) {
    return 1;
  } else {
    return -1;
  }
}

double median(const std::vector<double> &v) {
  // sort values
  std::vector<double> v_copy = v;
  std::sort(v_copy.begin(), v_copy.end());

  // obtain median
  if (v_copy.size() % 2 == 1) {
    // return middle value
    return v_copy[v_copy.size() / 2];

  } else {
    // grab middle two values and calc mean
    const double a = v_copy[v_copy.size() / 2];
    const double b = v_copy[(v_copy.size() / 2) - 1];
    return (a + b) / 2.0;
  }
}

double deg2rad(const double d) { return d * (M_PI / 180); }

double rad2deg(const double r) { return r * (180 / M_PI); }

void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 MatX &y) {
  int idx;

  // setup
  idx = 0;
  y.resize(rows, cols);

  // load matrix
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      y(j, i) = x[idx];
      idx++;
    }
  }
}

void load_matrix(const MatX &A, std::vector<double> &x) {
  for (int i = 0; i < A.cols(); i++) {
    for (int j = 0; j < A.rows(); j++) {
      x.push_back(A(j, i));
    }
  }
}

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

int euler2quat(const Vec3 &euler, const int euler_seq, Quaternion &q) {
  const double alpha = euler(0);
  const double beta = euler(1);
  const double gamma = euler(2);

  const double c1 = cos(alpha / 2.0);
  const double c2 = cos(beta / 2.0);
  const double c3 = cos(gamma / 2.0);
  const double s1 = sin(alpha / 2.0);
  const double s2 = sin(beta / 2.0);
  const double s3 = sin(gamma / 2.0);

  double w, x, y, z;
  switch (euler_seq) {
    case 123:
      // euler 1-2-3 to quaternion
      w = c1 * c2 * c3 - s1 * s2 * s3;
      x = s1 * c2 * c3 + c1 * s2 * s3;
      y = c1 * s2 * c3 - s1 * c2 * s3;
      z = c1 * c2 * s3 + s1 * s2 * c3;
      break;

    case 321:
      // euler 3-2-1 to quaternion
      w = c1 * c2 * c3 + s1 * s2 * s3;
      x = s1 * c2 * c3 - c1 * s2 * s3;
      y = c1 * s2 * c3 + s1 * c2 * s3;
      z = c1 * c2 * s3 - s1 * s2 * c3;
      break;

    default:
      LOG_ERROR("Error! Invalid euler sequence [%d]\n", euler_seq);
      return -1;
      break;
  }

  q.w() = w;
  q.x() = x;
  q.y() = y;
  q.z() = z;

  return 0;
}

int euler2rot(const Vec3 &euler, const int euler_seq, Mat3 &R) {
  double R11, R12, R13, R21, R22, R23, R31, R32, R33;
  double phi, theta, psi;

  phi = euler(0);
  theta = euler(1);
  psi = euler(2);

  if (euler_seq == 321) {
    // euler 3-2-1
    R11 = cos(theta) * cos(psi);
    R12 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R13 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);

    R21 = cos(theta) * sin(psi);
    R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R23 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);

    R31 = -sin(theta);
    R32 = sin(phi) * cos(theta);
    R33 = cos(phi) * cos(theta);

  } else if (euler_seq == 123) {
    // euler 1-2-3
    R11 = cos(theta) * cos(psi);
    R12 = cos(theta) * sin(psi);
    R13 = -sin(theta);

    R21 = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);
    R22 = sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi);
    R23 = sin(phi) * cos(theta);

    R31 = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
    R32 = cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
    R33 = cos(phi) * cos(theta);

  } else {
    return -1;
  }

  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;

  return 0;
}

int quat2euler(const Quaternion &q, const int euler_seq, Vec3 &euler) {
  double qw, qx, qy, qz;
  double qw2, qx2, qy2, qz2;
  double phi, theta, psi;

  qw = q.w();
  qx = q.x();
  qy = q.y();
  qz = q.z();

  qw2 = pow(qw, 2);
  qx2 = pow(qx, 2);
  qy2 = pow(qy, 2);
  qz2 = pow(qz, 2);

  if (euler_seq == 123) {
    // euler 1-2-3
    phi = atan2(2 * (qz * qw - qx * qy), (qw2 + qx2 - qy2 - qz2));
    theta = asin(2 * (qx * qz + qy * qw));
    psi = atan2(2 * (qx * qw - qy * qz), (qw2 - qx2 - qy2 + qz2));

  } else if (euler_seq == 321) {
    // euler 3-2-1
    phi = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
    theta = asin(2 * (qy * qw - qx * qz));
    psi = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  } else {
    return -1;
  }

  euler << phi, theta, psi;
  return 0;
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

  Mat3 R;
  R << R11, R12, R13, R21, R22, R23, R31, R32, R33;
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

void target2body(Vec3 target_pos_if,
                 Vec3 body_pos_if,
                 Quaternion body_orientation_if,
                 Vec3 &target_pos_bf) {
  // convert quaternion to rotation matrix
  const Mat3 R = body_orientation_if.toRotationMatrix().inverse();

  // calculate position difference and convert to body frame
  // assumes inertial frame is NWU
  const Vec3 pos_nwu = target_pos_if - body_pos_if;

  // compensate for body orientation by rotating
  target_pos_bf = R * pos_nwu;
}

void target2body(Vec3 target_pos_if,
                 Vec3 body_pos_if,
                 Vec3 body_orientation_if,
                 Vec3 &target_pos_bf) {
  // convert euler to rotation matrix
  Mat3 R;
  euler2rot(body_orientation_if, 123, R);

  // calculate position difference and convert to body frame
  // assumes inertial frame is NWU
  Vec3 pos_nwu = target_pos_if - body_pos_if;

  // compensate for body orientation by rotating
  target_pos_bf = R * pos_nwu;
}

void target2bodyplanar(Vec3 target_pos_if,
                       Vec3 body_pos_if,
                       Quaternion body_orientation_if,
                       Vec3 &target_pos_bf) {
  Vec3 euler;

  // convert quaternion to euler
  quat2euler(body_orientation_if, 123, euler);

  // filtering out roll and pitch since we are in body planar frame
  euler << 0.0, 0.0, euler(2);

  // calculate setpoint relative to quadrotor
  target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
}

void target2bodyplanar(Vec3 target_pos_if,
                       Vec3 body_pos_if,
                       Vec3 body_orientation_if,
                       Vec3 &target_pos_bf) {
  Vec3 euler;

  // filtering out roll and pitch since we are in body planar frame
  euler << 0.0, 0.0, body_orientation_if(2);

  // calculate setpoint relative to quadrotor
  target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
}

void target2inertial(Vec3 target_pos_bf,
                     Vec3 body_pos_if,
                     Vec3 body_orientation_if,
                     Vec3 &target_pos_if) {
  Mat3 R;

  // construct rotation matrix from euler
  euler2rot(body_orientation_if, 321, R);

  // transform target from body to inertial frame
  target_pos_if = (R * target_pos_bf) + body_pos_if;
}

void target2inertial(Vec3 target_pos_bf,
                     Vec3 body_pos_if,
                     Quaternion body_orientation_if,
                     Vec3 &target_pos_if) {
  // convert quaternion to rotation matrix
  const Mat3 R = body_orientation_if.toRotationMatrix();

  // transform target from body to inertial frame
  target_pos_if = (R * target_pos_bf) + body_pos_if;
}

void inertial2body(Vec3 nwu_if, Quaternion orientation_if, Vec3 &nwu_bf) {
  // create rotation matrix
  Vec3 rpy;
  quat2euler(orientation_if, 321, rpy);

  Mat3 R;
  euler2rot(rpy, 123, R);

  // transform inertal to body
  nwu_bf = R * nwu_if;
}

void inertial2body(Vec3 nwu_if, Vec3 orientation_if, Vec3 &nwu_bf) {
  // create rotation matrix
  Mat3 R;
  euler2rot(orientation_if, 123, R);

  // transform inertal to body
  nwu_bf = R * nwu_if;
}

double wrapTo180(const double euler_angle) {
  return fmod((euler_angle + 180.0), 360.0) - 180.0;
}

double wrapTo360(const double euler_angle) {
  if (euler_angle > 0) {
    return fmod(euler_angle, 360.0);
  } else {
    return fmod(euler_angle + 360, 360.0);
  }
}

double wrapToPi(const double r) { return deg2rad(wrapTo180(rad2deg(r))); }

double wrapTo2Pi(const double r) { return deg2rad(wrapTo360(rad2deg(r))); }

double cross_track_error(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos) {
  const double x0 = pos(0);
  const double y0 = pos(1);

  const double x1 = p1(0);
  const double y1 = p1(0);

  const double x2 = p2(0);
  const double y2 = p2(0);

  // calculate perpendicular distance between line (p1, p2) and point (pos)
  const double n = ((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
  const double d = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

  return fabs(n) / d;
}

int point_left_right(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
  const double a0 = a(0);
  const double a1 = a(1);
  const double b0 = b(0);
  const double b1 = b(1);
  const double c0 = c(0);
  const double c1 = c(1);
  const double x = (b0 - a0) * (c1 - a1) - (b1 - a1) * (c0 - a0);

  if (x > 0) {
    return 1; // left
  } else if (x < 0) {
    return 2; // right
  } else if (x == 0) {
    return 0; // parallel
  }

  return -1;
}

// int closest_point(Vec2 a, Vec2 b, Vec2 p, Vec2 &closest, bool limit) {
//   Vec2 v1, v2, result;
//   double t;
//
//   // pre-check
//   if ((a - b).norm() == 0) {
//     closest = a;
//     return -1;
//   }
//
//   // calculate closest point
//   v1 = p - a;
//   v2 = b - a;
//   t = v1.dot(v2) / v2.squaredNorm();
//
//   // check if point p is:
//   // 1. before point a
//   // 2. after point b
//   // 3. middle of point a, b
//   if (t < 0) {
//     closest = (limit) ? a : a + t * v2;
//     return 1;
//   } else if (t > 1) {
//     closest = (limit) ? b : a + t * v2;
//     return 2;
//   } else {
//     closest = a + t * v2;
//     return 0;
//   }
// }

double
closest_point(const Vec2 &a, const Vec2 &b, const Vec2 &p, Vec2 &closest) {
  // pre-check
  if ((a - b).norm() == 0) {
    closest = a;
    return -1;
  }

  // calculate closest point
  const Vec2 v1 = p - a;
  const Vec2 v2 = b - a;
  const double t = v1.dot(v2) / v2.squaredNorm();
  closest = a + t * v2;

  return t;
}

Vec2 linear_interpolation(const Vec2 &a, const Vec2 &b, const double mu) {
  return a * (1 - mu) + b * mu;
}

} // eof atl
