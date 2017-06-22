#ifndef __atl_ESTIMATION_EKF_HPP__
#define __atl_ESTIMATION_EKF_HPP__

#include "atl_core/utils/utils.hpp"


namespace atl {

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

}  // end of atl namespace
#endif
