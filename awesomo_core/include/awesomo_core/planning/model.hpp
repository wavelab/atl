#ifndef __AWESOMO_PLANNING_MODEL_HPP__
#define __AWESOMO_PLANNING_MODEL_HPP__

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class Quad2DModel {
public:
  bool configured;

  Vec4 x;
  double m;

  Quad2DModel(void);
  int configure(Vec4 x_init, double m);
  int update(Vec2 u, double dt);
  void printState(void);
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

  Simulator(void);
  int configure(Vec4 x_init, Vec4 x_final, double m);
  int simulate(double dt, double tend, MatX U, MatX &X);
};

}  // end of awesomo namespace
#endif
