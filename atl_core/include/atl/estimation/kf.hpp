#ifndef ATL_ESTIMATION_KF_HPP
#define ATL_ESTIMATION_KF_HPP

#include "atl/utils/utils.hpp"

namespace atl {

class KalmanFilter {
public:
  bool initialized;

  VecX mu;

  MatX B;
  MatX R;

  MatX C;
  MatX Q;

  MatX S;
  MatX I;
  MatX K;

  VecX mu_p;
  MatX S_p;

  KalmanFilter(void);
  int init(VecX mu, MatX R, MatX C, MatX Q);
  int reset(VecX mu, MatX R, MatX C, MatX Q);
  int estimate(MatX A, VecX y);
};

}  // namespace atl
#endif
