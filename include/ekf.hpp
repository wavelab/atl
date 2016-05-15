#ifndef __EKF_HPP__
#define __EKF_HPP__

#include <iostream>
#include <Eigen/Dense>


// STRUCTURES
struct ekf
{

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
int ekf_check(struct ekf *estimator);
void ekf_prediction_update(struct ekf *estimator, Eigen::VectorXd u, float dt);
void ekf_measurement_update(struct ekf *estimator, Eigen::VectorXd y, float dt);


// Attitude EKF
Eigen::VectorXd att_g_function(Eigen::VectorXd &mu_p, Eigen::VectorXd &u,
                               float dt);
Eigen::MatrixXd att_G_function(Eigen::VectorXd &mu_p, Eigen::VectorXd &u,
                               float dt);
Eigen::VectorXd att_h_function(Eigen::VectorXd &y, float dt);
Eigen::MatrixXd att_H_function(Eigen::VectorXd &mu_p, float dt);

int initilize_att_ekf(ekf *att_ekf, Eigen::MatrixXd &R, Eigen::MatrixXd &Q,
                      Eigen::VectorXd mu_init);

#endif
