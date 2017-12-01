#include "atl/estimation/ekf_tracker.hpp"

namespace atl {

int EKFTracker::configure(const std::string &config_file) {
  ConfigParser parser;
  std::string mode;

  // parse and load config file
  parser.addParam("nb_states", &this->nb_states);
  parser.addParam("motion_noise_matrix", &this->R);
  parser.addParam("measurement_noise_matrix", &this->Q);
  this->config_file = config_file;
  if (parser.load(config_file) != 0) {
    return -1;
  }

  this->configured = true;
  return 0;
}

int EKFTracker::initialize(const VecX &mu) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // initialize
  this->mu = mu;

  // this->R;  // set during configuration
  // this->Q;  // set during configuration

  this->S = MatX::Identity(this->nb_states, this->nb_states);
  this->I = MatX::Identity(this->nb_states, this->nb_states);
  // this->K;

  this->mu_p = VecX::Zero(this->nb_states);
  this->S_p = MatX::Zero(this->nb_states, this->nb_states);

  this->initialized = true;
  return 0;
}

int EKFTracker::reset(const VecX &mu) {
  // configure
  if (this->configure(this->config_file) != 0) {
    this->configured = false;
    return -1;
  }

  // initialize
  if (this->initialize(mu) != 0) {
    this->initialized = false;
    return -2;
  }

  return 0;
}

int EKFTracker::predictionUpdate(const VecX &g, const MatX &G) {
  // pre-check
  if (this->initialized == false) {
    return -1;
  }

  // prediction update
  mu_p = g;
  S_p = G * S * G.transpose() + R;

  return 0;
}

int EKFTracker::measurementUpdate(const VecX &h, const MatX &H, const VecX &y) {
  // pre-check
  if (this->initialized == false) {
    return -1;
  }

  // measurement update
  K = S_p * H.transpose() * (H * S_p * H.transpose() + Q).inverse();
  mu = mu_p + K * (y - h);
  S = (I - K * H) * S_p;

  return 0;
}

void two_wheel_process_model(EKFTracker &ekf, MatX &G, VecX &g, double dt) {
  // x0 - x
  // x1 - y
  // x2 - z
  // x3 - theta
  // x4 - v
  // x5 - vz
  // x6 - omega
  // x7 - a
  // x8 - az

  // g - proces model
  g(0) = ekf.mu(0) + ekf.mu(4) * cos(ekf.mu(3)) * dt;
  g(1) = ekf.mu(1) + ekf.mu(4) * sin(ekf.mu(3)) * dt;
  g(2) = ekf.mu(2) + ekf.mu(5) * dt;
  g(3) = ekf.mu(3) + ekf.mu(6) * dt;
  g(4) = ekf.mu(4) + ekf.mu(7) * dt;
  g(5) = ekf.mu(5) + ekf.mu(8) * dt;
  g(6) = ekf.mu(6);
  g(7) = ekf.mu(7);
  g(8) = ekf.mu(8);

  // G - linearized process model
  // clang-format off
  G << 1, 0, 0, -dt * ekf.mu(4) * sin(ekf.mu(3)), dt * cos(ekf.mu(3)), 0,  0,  0,  0,
       0, 1, 0, dt * ekf.mu(4) * cos(ekf.mu(3)), dt * sin(ekf.mu(3)), 0,  0,  0,  0,
       0, 0, 1, 0, 0, dt, 0, 0, 0,
       0, 0, 0, 1, 0, 0, dt, 0, 0,
       0, 0, 0, 0, 1, 0, 0, dt, 0,
       0, 0, 0, 0, 0, 1, 0, 0, dt,
       0, 0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 1;
  // clang-format on
}

void two_wheel_measurement_model(EKFTracker &ekf, MatX &H, VecX &h) {
  H(0, 0) = 1.0; // x
  H(1, 1) = 1.0; // y
  H(2, 2) = 1.0; // z
  H(3, 3) = 1.0; // theta
  h = H * ekf.mu_p;
}

} // namespace atl
