#include "ekf.hpp"


void ekf_prediction_update(
    struct ekf *e,
    Eigen::VectorXd (*g_function)(Eigen::VectorXd &, Eigen::VectorXd &, double),
    Eigen::VectorXd (*G_function)(Eigen::VectorXd &, Eigen::VectorXd &, double),
    Eigen::VectorXd u
)
{
    Eigen::MatrixXd g;
    Eigen::MatrixXd G;

    g = e->g_function(u, e->mu, e->dt);
    G = e->G_function(u, e->mu, e->dt);

    e->mu_p = g;
    e->S_p = G * e->S * G.transpose() + e->R;
}

void ekf_measurement_update(
    struct ekf *e,
    Eigen::VectorXd (*h_function)(Eigen::VectorXd &, double),
    Eigen::VectorXd (*H_function)(Eigen::VectorXd &, double),
    Eigen::VectorXd y
)
{
    Eigen::MatrixXd h;
    Eigen::MatrixXd H;

    h = e->h_function(e->mu_p, e->dt);
    H = e->H_function(e->mu_p, e->dt);

    // K = e->S_p * H.transpose() * inv(H * e->S_p * H.transpose() + e->Q);
    e->mu = e->mu_p + e->K * (y - h);
    // e->S = (eye(length(e->mu)) - K * H) * e->S_p;
}
