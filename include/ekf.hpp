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

    Eigen::VectorXd (*g_function)(Eigen::VectorXd &, Eigen::VectorXd &, double);
    Eigen::VectorXd (*G_function)(Eigen::VectorXd &, Eigen::VectorXd &, double);
    Eigen::VectorXd (*h_function)(Eigen::VectorXd &, double);
    Eigen::VectorXd (*H_function)(Eigen::VectorXd &, double);
};


// FUNCTIONS
void ekf_prediction_update(struct ekf *estimator, g_func, G_func, u);
void ekf_measurement_update(struct ekf *estimator, h_func, H_func, y)

#endif
