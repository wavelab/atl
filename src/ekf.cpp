#include "ekf.hpp"


void ekf_prediction_update(
    struct ekf *e,
    Eigen::VectorXd (*g_function)(Eigen::VectorXd &, Eigen::VectorXd &, double),
    Eigen::MatrixXd (*G_function)(Eigen::VectorXd &, Eigen::VectorXd &, double),
    Eigen::VectorXd u
)
{
    Eigen::MatrixXd g;
    Eigen::MatrixXd G;

    // g = e->g_function(u, e->mu, e->dt);
    // G = e->G_function(u, e->mu, e->dt);

    g = g_function(u, e->mu, e->dt);
    G = G_function(u, e->mu, e->dt);

    std::cout << G << std::endl;
    e->mu_p = g;
    std::cout << e->R << std::endl;
    e->S_p = G * e->S * G.transpose(); // + e->R;
    std::cout << e->S_p << std::endl;
}

void ekf_measurement_update(
    struct ekf *e,
    Eigen::VectorXd (*h_function)(Eigen::VectorXd &, double),
    Eigen::MatrixXd (*H_function)(Eigen::VectorXd &, double),
    Eigen::VectorXd y
)
{
    Eigen::VectorXd h;
    Eigen::MatrixXd H;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(e->mu.size(), e->mu.size());


    // had to remove e->h_function from all of these? Chris halp
    h = h_function(e->mu_p, e->dt);
    H = H_function(e->mu_p, e->dt);


    e->K = e->S_p * H.transpose() * (H * e->S_p * H.transpose() + e->Q).inverse();
    e->mu = e->mu_p + e->K * (y - h);

    e->S = (I - e->K * H) *  e->S_p;
    std::cout << e->S << std::endl;
}
