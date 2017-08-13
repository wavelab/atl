#ifndef ATL_PLANNING_MODEL_HPP
#define ATL_PLANNING_MODEL_HPP

#include "atl/utils/utils.hpp"

namespace atl {

class Quad2DModel {
public:
  bool configured;

  Vec4 x;
  double m;

  Quad2DModel();
  int configure(Vec4 x_init, double m);
  int update(Vec2 u, double dt);
  void printState();
};

class Simulator {
public:
  bool configured;

  Quad2DModel model;
  Vec4 x_init;
  Vec4 x_final;

  double d_az;
  double d_theta;
  double az_sum;
  double dist_error;
  double vel_error;

  Simulator();
  int configure(Vec4 x_init, Vec4 x_final, double m);
  int simulate(double dt, double tend, MatX U, MatX &X);
};

} // namespace atl
#endif
