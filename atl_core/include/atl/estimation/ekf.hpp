#ifndef ATL_ESTIMATION_EKF_HPP
#define ATL_ESTIMATION_EKF_HPP

#include "atl/utils/utils.hpp"

namespace atl {

class EKF {
public:
  bool initialized = false;
  VecX mu;

  MatX R;
  MatX Q;

  MatX S;
  MatX I;
  MatX K;

  VecX mu_p;
  MatX S_p;

  EKF() {}

  /**
   * Initialize
   *
   * @param mu Initial estimate
   * @param R Motion noise matrix
   * @param Q Sensor noise matrix
   *
   * @return 0 for success, -1 for failure
   */
  int init(VecX mu, MatX R, MatX Q);

  /**
   * Prediction update
   *
   * @param g motion vector
   * @param G Linearized motion vector
   *
   * @return
   *    - 0: Success
   *    - -1: Not intiailized
   */
  int predictionUpdate(VecX g, MatX G);

  /**
   * Measurement update
   *
   * @param h Inverse measurement
   * @param H Linearized inverse measurement
   * @param y Measurement
   *
   * @return
   *    - 0: Success
   *    - -1: Not intiailized
   */
  int measurementUpdate(VecX h, MatX H, VecX y);
};

} // namespace atl
#endif
