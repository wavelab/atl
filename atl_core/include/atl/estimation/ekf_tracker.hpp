#ifndef ATL_ESTIMATION_EKF_TRACKER_HPP
#define ATL_ESTIMATION_EKF_TRACKER_HPP

#include "atl/utils/utils.hpp"

#define TWO_WHEEL_MOTION_MODEL(EKF, G, g)         \
  G << 1, 0, (-u(0) * sin(EKF.mu(2)) * dt), 0, 1, \
    (u(0) * cos(EKF.mu(2)) * dt), 0, 0, 1;        \
  g << EKF.mu(0) + u(0) * cos(EKF.mu(2)) * dt,    \
    EKF.mu(1) + u(0) * sin(EKF.mu(2)) * dt, EKF.mu(2) + u(1) * dt;

#define TWO_WHEEL_MEASUREMENT_MODEL(EKF, H, h) \
  H = MatX::Identity(3, 3);                    \
  h = H * EKF.mu;

#define TWO_WHEEL_NO_INPUTS_MOTION_MODEL(EKF, G, g, pn1, pn2)                \
  G << 1, 0, (-pn1 * sin(EKF.mu(2)) * dt), cos(EKF.mu(2)) * dt, 0, 0, 1,     \
    (pn1 * cos(EKF.mu(2)) * dt), sin(EKF.mu(2)) * dt, 0, 0, 0, 1, 0, dt, 0,  \
    0, 0, 1, 0, 0, 0, 0, 0, 1;                                               \
  g << EKF.mu(0) + EKF.mu(3) * cos(EKF.mu(2)) * dt,                          \
    EKF.mu(1) + EKF.mu(3) * sin(EKF.mu(2)) * dt, EKF.mu(2) + EKF.mu(4) * dt, \
    EKF.mu(3) + pn1, EKF.mu(4) + pn2;

#define TWO_WHEEL_NO_INPUTS_MEASUREMENT_MODEL(EKF, H, h) \
  H = MatX::Identity(5, 5);                              \
  h = H * EKF.mu;

#define TWO_WHEEL_3D_NO_INPUTS_MOTION_MODEL(EKF, G, g)                       \
  G << 1, 0, 0, (-EKF.mu(4) * sin(EKF.mu(3)) * dt), cos(EKF.mu(3)) * dt, 0,  \
    0, 0, 1, 0, (EKF.mu(4) * cos(EKF.mu(3)) * dt), sin(EKF.mu(3)) * dt, 0,   \
    0, 0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 1, 0, dt, 0, 0, 0, 0, 0, 1, 0, 0, 0,   \
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;                                   \
  g << EKF.mu(0) + EKF.mu(4) * cos(EKF.mu(3)) * dt,                          \
    EKF.mu(1) + EKF.mu(4) * sin(EKF.mu(3)) * dt, EKF.mu(2) + EKF.mu(6) * dt, \
    EKF.mu(3) + EKF.mu(5) * dt, EKF.mu(4), EKF.mu(5), EKF.mu(6);

#define TWO_WHEEL_3D_NO_INPUTS_MEASUREMENT_MODEL(EKF, H, h) \
  H = MatX::Zero(4, 7);                                     \
  H(0, 0) = 1.0; /* x */                                    \
  H(1, 1) = 1.0; /* y */                                    \
  H(2, 2) = 1.0; /* z */                                    \
  H(3, 3) = 1.0; /* theta */                                \
  h = H * EKF.mu;

namespace atl {

class ExtendedKalmanFilterTracker {
public:
  bool configured;
  bool initialized;

  int nb_states;
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

void two_wheel_process_model(ExtendedKalmanFilterTracker &ekf,
                             MatX &G,
                             VecX &g,
                             double dt);

void two_wheel_measurement_model(ExtendedKalmanFilterTracker &ekf,
                                 MatX &H,
                                 VecX &h);

}  // namespace atl
#endif
