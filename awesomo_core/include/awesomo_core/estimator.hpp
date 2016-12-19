#ifndef __KF_HPP__
#define __KF_HPP__

#include <iostream>


#include <Eigen/Geometry>


// STRUCTURES
struct kf {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd R;

  Eigen::MatrixXd C;
  Eigen::MatrixXd Q;

  Eigen::MatrixXd S;
  Eigen::MatrixXd I;
  Eigen::MatrixXd K;

  Eigen::VectorXd mu;

  Eigen::VectorXd mu_p;
  Eigen::MatrixXd S_p;
};

struct ekf {
  Eigen::VectorXd mu;
  Eigen::MatrixXd S;
  Eigen::MatrixXd R;
  Eigen::MatrixXd Q;

  Eigen::VectorXd mu_p;
  Eigen::MatrixXd S_p;
  Eigen::MatrixXd K;
  Eigen::MatrixXd I;

  Eigen::VectorXd (*g_function)(Eigen::VectorXd &, Eigen::VectorXd &, float);
  Eigen::MatrixXd (*G_function)(Eigen::VectorXd &, Eigen::VectorXd &, float);
  Eigen::VectorXd (*h_function)(Eigen::VectorXd &, float);
  Eigen::MatrixXd (*H_function)(Eigen::VectorXd &, float);
};


// FUNCTIONS
void apriltag_kf_setup(struct kf *e, Eigen::VectorXd mu);
void apriltag_kf_estimate(struct kf *e,
                          Eigen::VectorXd y,
                          float dt,
                          bool tag_detected);

void ekf_prediction_update(struct ekf *e, Eigen::VectorXd u, float dt);
void ekf_measurement_update(struct ekf *e, Eigen::VectorXd y, float dt);

#endif
