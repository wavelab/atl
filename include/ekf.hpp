#ifndef __EKF_HPP__
#define __EKF_HPP__

#include <iostream>

#include <Eigen/Dense>


// STRUCTURES
struct ekf
{
    double dt;

    Eigen::VectorXd mu;
    Eigen::MatrixXd S;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;

    Eigen::VectorXd mu_p;
    Eigen::MatrixXd S_p;
    Eigen::MatrixXd K;
    Eigen::MatrixXd I;

    Eigen::VectorXd (*g_function)(Eigen::VectorXd &, Eigen::VectorXd &, double);
    Eigen::MatrixXd (*G_function)(Eigen::VectorXd &, Eigen::VectorXd &, double);
    Eigen::VectorXd (*h_function)(Eigen::VectorXd &, double);
    Eigen::MatrixXd (*H_function)(Eigen::VectorXd &, double);
};


// FUNCTIONS
int ekf_check(struct ekf *estimator);
void ekf_prediction_update(struct ekf *estimator, Eigen::VectorXd u);
void ekf_measurement_update(struct ekf *estimator, Eigen::VectorXd y);

#endif
