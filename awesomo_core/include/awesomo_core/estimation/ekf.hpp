#ifndef __AWESOMO_ESTIMATION_EKF_HPP__
#define __AWESOMO_ESTIMATION_EKF_HPP__

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class ExtendedKalmanFilter {
  VecX mu;
  MatX S;
  MatX R;
  MatX Q;

  VecX mu_p;
  MatX S_p;
  MatX K;
  MatX I;

  VecX (*g_function)(VecX &, VecX &, float);
  MatX (*G_function)(VecX &, VecX &, float);
  VecX (*h_function)(VecX &, float);
  MatX (*H_function)(VecX &, float);

  void predictionUpdate(VecX u, float dt);
  void measurementUpdate(VecX y, float dt);
};

}  // end of awesomo namespace
