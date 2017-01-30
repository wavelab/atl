#include "awesomo_core/estimation/ekf_tracker.hpp"


namespace awesomo {

ExtendedKalmanFilterTracker::ExtendedKalmanFilterTracker(void) {
  this->configured = false;
  this->initialized = false;

  this->nb_states = 0;
  this->nb_dimensions = 0;
  this->config_file = "";

  this->mu = VecX::Zero(1);

  this->R = MatX::Zero(1, 1);
  this->Q = MatX::Zero(1, 1);

  this->S = MatX::Zero(1, 1);
  this->I = MatX::Zero(1, 1);
  this->K = MatX::Zero(1, 1);

  this->mu_p = VecX::Zero(1);
  this->S_p = MatX::Zero(1, 1);
}

int ExtendedKalmanFilterTracker::configure(std::string config_file) {
  ConfigParser parser;
  std::string mode;

  // parse and load config file
  parser.addParam<int>("nb_states", &this->nb_states);
  parser.addParam<int>("nb_dimensions", &this->nb_dimensions);
  parser.addParam<MatX>("motion_noise_matrix", &this->R);
  parser.addParam<MatX>("measurement_noise_matrix", &this->Q);
  this->config_file = config_file;
  if (parser.load(config_file) != 0) {
    return -1;
  }

  this->configured = true;
  return 0;
}

int ExtendedKalmanFilterTracker::initialize(VecX mu) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // initialize
  this->mu = mu;

  this->R;  // set during configuration
  this->Q;  // set during configuration

  this->S = MatX::Identity(this->nb_states, this->nb_states);
  this->I = MatX::Identity(this->nb_states, this->nb_states);
  this->K = MatX::Zero(this->nb_states, this->nb_states);

  this->mu_p = VecX::Zero(this->nb_states);
  this->S_p = MatX::Zero(this->nb_states, this->nb_states);

  this->initialized = true;
  return 0;
}

int ExtendedKalmanFilterTracker::reset(VecX mu) {
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
}

int ExtendedKalmanFilterTracker::predictionUpdate(VecX g, MatX G) {
  // pre-check
  if (this->initialized == false) {
    return -1;
  }

  // prediction update
  mu_p = g;
  S_p = G * S * G.transpose() + R;

  return 0;
}

int ExtendedKalmanFilterTracker::measurementUpdate(VecX h, MatX H, VecX y) {
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

}  // end of awesomo namespace
