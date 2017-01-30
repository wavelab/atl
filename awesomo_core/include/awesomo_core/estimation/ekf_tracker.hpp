#ifndef __AWESOMO_ESTIMATION_EKF_TRACKER_HPP__
#define __AWESOMO_ESTIMATION_EKF_TRACKER_HPP__

#include "awesomo_core/utils/utils.hpp"


#define TWO_WHEEL_MOTION_MODEL(EKF, G, g)       \
  G << 1, 0, (-u(0) * sin(EKF.mu(2)) * dt),     \
        0, 1, (u(0) * cos(EKF.mu(2)) * dt),     \
        0, 0, 1;                                \
  g << EKF.mu(0) + u(0) * cos(EKF.mu(2)) * dt,  \
        EKF.mu(1) + u(0) * sin(EKF.mu(2)) * dt, \
        EKF.mu(2) + u(1) * dt;

#define TWO_WHEEL_MEASUREMENT_MODEL(EKF, H, h) \
    H = MatX::Identity(3, 3);                  \
    h = H * EKF.mu;

namespace awesomo {

class ExtendedKalmanFilterTracker {
public:
  bool configured;
  bool initialized;

  int nb_states;
  int nb_dimensions;
  std::string config_file;

  VecX mu;

  MatX R;
  MatX Q;

  MatX S;
  MatX I;
  MatX K;

  VecX mu_p;
  MatX S_p;

  ExtendedKalmanFilterTracker(void);
  int configure(std::string config_file);
  int initialize(VecX mu);
  int reset(VecX mu);
  int predictionUpdate(VecX g, MatX G);
  int measurementUpdate(VecX h, MatX H, VecX y);
};

}  // end of awesomo namespace
#endif
