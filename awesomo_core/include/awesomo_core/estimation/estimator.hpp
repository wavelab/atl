#ifndef __AWESOMO_ESTIMATION_ESTIMATOR_HPP__
#define __AWESOMO_ESTIMATION_ESTIMATOR_HPP__

#include <iostream>

#include <Eigen/Geometry>

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

struct kf {
  MatX A;
  MatX B;
  MatX R;

  MatX C;
  MatX Q;

  MatX S;
  MatX I;
  MatX K;

  VecX mu;

  VecX mu_p;
  MatX S_p;
};

struct ekf {
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
};

void apriltag_kf_setup(struct kf *e, VecX mu);
void apriltag_kf_estimate(struct kf *e, VecX y, float dt, bool tag_detected);
void ekf_prediction_update(struct ekf *e, VecX u, float dt);
void ekf_measurement_update(struct ekf *e, VecX y, float dt);

}  // end of awesomo namespace
#endif
