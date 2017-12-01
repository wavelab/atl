#include "atl/estimation/ekf.hpp"

namespace atl {

int EKF::init(VecX mu, MatX R, MatX Q) {
  int nb_states;

  nb_states = mu.size();
  this->initialized = true;
  this->mu = mu;

  this->R = R;
  this->Q = Q;

  this->S = MatX::Identity(nb_states, nb_states);
  this->I = MatX::Identity(nb_states, nb_states);
  this->K = MatX::Zero(nb_states, nb_states);

  this->mu_p = VecX::Zero(nb_states);
  this->S_p = MatX::Zero(nb_states, nb_states);

  return 0;
}

int EKF::predictionUpdate(VecX g, MatX G) {
  // pre-check
  if (this->initialized == false) {
    return -1;
  }

  // prediction update
  mu_p = g;
  S_p = G * S * G.transpose() + R;

  return 0;
}

int EKF::measurementUpdate(VecX h, MatX H, VecX y) {
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

} // namespace atl
