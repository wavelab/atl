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

// void target2body(Vec3 target_pos_if,
//                  Vec3 body_pos_if,
//                  Quaternion body_orientation_if,
//                  Vec3 &target_pos_bf) {
//   // convert quaternion to rotation matrix
//   const Mat3 R = body_orientation_if.toRotationMatrix().inverse();
//
//   // calculate position difference and convert to body frame
//   // assumes inertial frame is NWU
//   const Vec3 pos_nwu = target_pos_if - body_pos_if;
//
//   // compensate for body orientation by rotating
//   target_pos_bf = R * pos_nwu;
// }
//
// void target2body(Vec3 target_pos_if,
//                  Vec3 body_pos_if,
//                  Vec3 body_orientation_if,
//                  Vec3 &target_pos_bf) {
//   // convert euler to rotation matrix
//   const Mat3 R = euler123ToRot(body_orientation_if);
//
//   // calculate position difference and convert to body frame
//   // assumes inertial frame is NWU
//   const Vec3 pos_nwu = target_pos_if - body_pos_if;
//
//   // compensate for body orientation by rotating
//   target_pos_bf = R * pos_nwu;
// }
//
// void target2bodyplanar(Vec3 target_pos_if,
//                        Vec3 body_pos_if,
//                        Quaternion body_orientation_if,
//                        Vec3 &target_pos_bf) {
//   // convert quaternion to euler
//   Vec3 euler = quatToEuler123(body_orientation_if);
//
//   // filtering out roll and pitch since we are in body planar frame
//   euler << 0.0, 0.0, euler(2);
//
//   // calculate setpoint relative to quadrotor
//   target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
// }
//
// void target2bodyplanar(Vec3 target_pos_if,
//                        Vec3 body_pos_if,
//                        Vec3 body_orientation_if,
//                        Vec3 &target_pos_bf) {
//   // filtering out roll and pitch since we are in body planar frame
//   const Vec3 euler{0.0, 0.0, body_orientation_if(2)};
//
//   // calculate setpoint relative to quadrotor
//   target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
// }

// void target2inertial(Vec3 target_pos_bf,
//                      Vec3 body_pos_if,
//                      Vec3 body_orientation_if,
//                      Vec3 &target_pos_if) {
//   // construct rotation matrix from euler
//   const Mat3 R = euler321ToRot(body_orientation_if);
//
//   // transform target from body to inertial frame
//   target_pos_if = (R * target_pos_bf) + body_pos_if;
// }
//
// void target2inertial(Vec3 target_pos_bf,
//                      Vec3 body_pos_if,
//                      Quaternion body_orientation_if,
//                      Vec3 &target_pos_if) {
//   // convert quaternion to rotation matrix
//   const Mat3 R = body_orientation_if.toRotationMatrix();
//
//   // transform target from body to inertial frame
//   target_pos_if = (R * target_pos_bf) + body_pos_if;
// }

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

Vec2 lerp(const Vec2 &a, const Vec2 &b, const double mu) {
  return a * (1 - mu) + b * mu;
}

} // eof atl
