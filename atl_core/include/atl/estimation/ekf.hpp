#ifndef ATL_ESTIMATION_EKF_HPP
#define ATL_ESTIMATION_EKF_HPP

#include "atl/utils/utils.hpp"

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

}  // namespace atl
#endif
