#ifndef __AWESOMO_ESTIMATION_EKF_HPP__
#define __AWESOMO_ESTIMATION_EKF_HPP__

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class ExtendedKalmanFilter {
public:
  bool initialized;
  VecX mu;

  MatX R;
  MatX Q;

  MatX S;
  MatX I;
  MatX K;

  VecX mu_p;
  MatX S_p;

  ExtendedKalmanFilter(void);
  int init(VecX mu, MatX R, MatX Q);
  int predictionUpdate(VecX g, MatX G);
  int measurementUpdate(VecX h, MatX H, VecX y);
};

}  // end of awesomo namespace
#endif
