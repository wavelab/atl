#ifndef ATL_ESTIMATION_EKF_TRACKER_HPP
#define ATL_ESTIMATION_EKF_TRACKER_HPP

#include "atl/utils/utils.hpp"

#define TWO_WHEEL_MOTION_MODEL(EKF, G, g)                                      \
  G << 1, 0, (-u(0) * sin(EKF.mu(2)) * dt), 0, 1,                              \
      (u(0) * cos(EKF.mu(2)) * dt), 0, 0, 1;                                   \
  g << EKF.mu(0) + u(0) * cos(EKF.mu(2)) * dt,                                 \
      EKF.mu(1) + u(0) * sin(EKF.mu(2)) * dt, EKF.mu(2) + u(1) * dt;

#define TWO_WHEEL_MEASUREMENT_MODEL(EKF, H, h)                                 \
  H = MatX::Identity(3, 3);                                                    \
  h = H * EKF.mu;

#define TWO_WHEEL_NO_INPUTS_MOTION_MODEL(EKF, G, g, pn1, pn2)                  \
  G << 1, 0, (-pn1 * sin(EKF.mu(2)) * dt), cos(EKF.mu(2)) * dt, 0, 0, 1,       \
      (pn1 * cos(EKF.mu(2)) * dt), sin(EKF.mu(2)) * dt, 0, 0, 0, 1, 0, dt, 0,  \
      0, 0, 1, 0, 0, 0, 0, 0, 1;                                               \
  g << EKF.mu(0) + EKF.mu(3) * cos(EKF.mu(2)) * dt,                            \
      EKF.mu(1) + EKF.mu(3) * sin(EKF.mu(2)) * dt, EKF.mu(2) + EKF.mu(4) * dt, \
      EKF.mu(3) + pn1, EKF.mu(4) + pn2;

#define TWO_WHEEL_NO_INPUTS_MEASUREMENT_MODEL(EKF, H, h)                       \
  H = MatX::Identity(5, 5);                                                    \
  h = H * EKF.mu;

#define TWO_WHEEL_3D_NO_INPUTS_MOTION_MODEL(EKF, G, g)                         \
  G << 1, 0, 0, (-EKF.mu(4) * sin(EKF.mu(3)) * dt), cos(EKF.mu(3)) * dt, 0, 0, \
      0, 1, 0, (EKF.mu(4) * cos(EKF.mu(3)) * dt), sin(EKF.mu(3)) * dt, 0, 0,   \
      0, 0, 1, 0, 0, 0, dt, 0, 0, 0, 1, 0, dt, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,   \
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;                                      \
  g << EKF.mu(0) + EKF.mu(4) * cos(EKF.mu(3)) * dt,                            \
      EKF.mu(1) + EKF.mu(4) * sin(EKF.mu(3)) * dt, EKF.mu(2) + EKF.mu(6) * dt, \
      EKF.mu(3) + EKF.mu(5) * dt, EKF.mu(4), EKF.mu(5), EKF.mu(6);

#define TWO_WHEEL_3D_NO_INPUTS_MEASUREMENT_MODEL(EKF, H, h)                    \
  H = MatX::Zero(4, 7);                                                        \
  H(0, 0) = 1.0; /* x */                                                       \
  H(1, 1) = 1.0; /* y */                                                       \
  H(2, 2) = 1.0; /* z */                                                       \
  H(3, 3) = 1.0; /* theta */                                                   \
  h = H * EKF.mu;

namespace atl {

class EKFTracker {
public:
  bool configured = false;
  bool initialized = false;

  int nb_states = 0;
  std::string config_file;

  VecX mu = VecX::Zero(1);

  MatX R = MatX::Zero(1, 1);
  MatX Q = MatX::Zero(1, 1);

  MatX S = MatX::Zero(1, 1);
  MatX I = MatX::Zero(1, 1);
  MatX K = MatX::Zero(1, 1);

  VecX mu_p = MatX::Zero(1, 1);
  MatX S_p = MatX::Zero(1, 1);

  EKFTracker() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Initialize
   *
   * @param mu Initial estimate
   * @return 0 for success, -1 for failure
   */
  int initialize(const VecX &mu);

  /**
   * Reset estimator with new estimates
   *
   * @param mu Estimate
   * @return 0 for success, -1 for failure
   */
  int reset(const VecX &mu);

  /**
   * Prediction update
   *
   * @param g motion vector
   * @param G Linearized motion vector
   *
   * @return
   *    - 0: Success
   *    - -1: Not initialized
   */
  int predictionUpdate(const VecX &g, const MatX &G);

  /**
   * Measurement update
   *
   * @param h Inverse measurement
   * @param H Linearized inverse measurement
   * @param y Measurement
   *
   * @return
   *    - 0: Success
   *    - -1: Not initialized
   */
  int measurementUpdate(const VecX &h, const MatX &H, const VecX &y);
};

void two_wheel_process_model(EKFTracker &ekf, MatX &G, VecX &g, double dt);

void two_wheel_measurement_model(EKFTracker &ekf, MatX &H, VecX &h);

} // namespace atl
#endif
