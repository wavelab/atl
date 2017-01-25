#include "awesomo_core/planning/trajectory.hpp"


namespace awesomo {

// Vec2 quadrotor_calculate_inputs(double mass, double thrust, double omega) {
//   Vec2 u;
//
//   u(0) = thrust / mass;
//   u(1) = omega;
//
//   return u;
// }

VecX quadrotor_2d_model(VecX x, Vec2 u) {
  const double k_gravity = 9.81;

  // x1 - x
  // x2 - z
  // x3 - vx
  // x4 - vz
  // x5 - theta (pitch)

  x(0) = x(0) + x(1);
  x(1) = x(1) + u(0) * sin(x(4));
  x(2) = x(2) + x(3);
  x(3) = x(3) + u(0) * cos(x(4)) - k_gravity;
  x(4) = x(4) + u(1);

  return x;
}

// double input1_constraint(const std::vector<double> &x,
//                          std::vector<double> &grad,
//                          void *data) {
//
//
// }
//
// double input2_constraint(const std::vector<double> &x,
//                          std::vector<double> &grad,
//                          void *data) {
//
//
// }

}  // end of awesomo namespace
