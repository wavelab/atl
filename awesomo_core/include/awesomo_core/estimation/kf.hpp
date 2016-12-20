#ifndef __AWESOMO_ESTIMATION_KF_HPP__
#define __AWESOMO_ESTIMATION_KF_HPP__

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class KalmanFilter {
public:
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

  KalmanFilter(void);
  KalmanFilter(VecX mu);
  void estimate(VecX y, float dt, bool tag_detected);
};

}  // end of awesomo namespace
#endif
