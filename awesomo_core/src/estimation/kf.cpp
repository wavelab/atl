#include "awesomo_core/estimation/kf.hpp"


namespace awesomo {

KalmanFilter::KalmanFilter(void) {
  this->A = MatX(9, 9);
  this->B = MatX(9, 9);
  this->R = MatX(9, 9);

  this->C = MatX(3, 9);
  this->Q = MatX(3, 3);

  this->S = MatX(9, 9);
  this->I = MatX(9, 9);
  this->K = MatX(9, 9);

  this->mu_p = VecX(9);
  this->S_p = MatX(9, 9);
}

KalmanFilter::KalmanFilter(VecX mu) {
  MatX A(9, 9);
  MatX B(9, 9);
  MatX R(9, 9);

  MatX C(3, 9);
  MatX Q(3, 3);

  MatX S(9, 9);
  MatX I(9, 9);
  MatX K(9, 9);

  VecX mu_p(9);
  MatX S_p(9, 9);

  // transition matrix (assuming constant acceleration)
  // clang-format off
  A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // input matrix
  B = MatX::Zero(9, 9);

  // motion noise
  // clang-format off
  R << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // measurement model
  // clang-format off
  C << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // clang-format on

  // measurement noise
  // clang-format off
  Q << 20.0, 0.0, 0.0,
       0.0, 20.0, 0.0,
       0.0, 0.0, 20.0;
  // clang-format on

  // misc
  S = MatX::Identity(9, 9) * 100;
  I = MatX::Identity(9, 9);
  K = MatX::Zero(9, 9);
  // mu_p = VecX::Zero(9);
  S_p = MatX::Zero(9, 9);

  // configure kalman filter
  this->mu = mu;

  this->A = A;
  this->B = B;
  this->R = R;

  this->C = C;
  this->Q = Q;

  this->S = S;
  this->I = I;
  this->K = K;

  this->mu_p = mu_p;
  this->S_p = S_p;
}

void KalmanFilter::estimate(VecX y, float dt, bool tag_detected) {
  // transition matrix (constant acceleration)
  // // clang-format on
  this->A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2) / 2.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, dt, 0.0, 0.0, pow(dt, 2) / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, dt, 0.0, 0.0, pow(dt, 2) / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0;
  // clang-format on

  // transition matrix (constant velocity)
  // // clang-format off
  // this->A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
  //         0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  //         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // // clang-format on

  // prediction update
  this->mu_p = this->A * this->mu;
  this->S_p = this->A * this->S * this->A.transpose() + this->R;

  // measurement update
  if (tag_detected) {
    // clang-format off
    this->K = this->S_p * this->C.transpose() * (this->C * this->S_p * this->C.transpose() + this->Q).inverse();
    this->mu = this->mu_p + this->K * (y - this->C * this->mu_p);
    this->S = (this->I - this->K * this->C) * this->S_p;
    // clang-format on

  } else {
    this->mu = this->mu_p;
    this->S = this->S_p;
  }
}

}  // end of awesomo namespace
