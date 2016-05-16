#include "ekf.hpp"


void ekf_prediction_update(struct ekf *e, Eigen::VectorXd u, float dt)
{
    Eigen::MatrixXd g;
    Eigen::MatrixXd G;

    g = e->g_function(u, e->mu, dt);
    G = e->G_function(u, e->mu, dt);

    e->mu_p = g;
    e->S_p = G * e->S * G.transpose() + e->R;
}

void ekf_measurement_update(struct ekf *e, Eigen::VectorXd y, float dt)
{
    Eigen::VectorXd h;
    Eigen::MatrixXd H;

    h = e->h_function(e->mu_p, dt);
    H = e->H_function(e->mu_p, dt);

    e->K = e->S_p * H.transpose() * (H * e->S_p * H.transpose() + e->Q).inverse();
    e->mu = e->mu_p + e->K * (y - h);

    e->S = (e->I - e->K * H) *  e->S_p;
}




// Attitude Ekf functions
Eigen::VectorXd ekf_attitude_g(
    Eigen::VectorXd &mu,
    Eigen::VectorXd &u,
    float dt
)
{
    // work with only roll, pitch, yaw and first order derivatives
    // mu[0] = roll
    // mu[1] = pitch
    // mu[2] = yaw
    // mu[3] = roll_dot
    // mu[4] = pitch_dot
    // mu[5] = yaw_dot

    Eigen::VectorXd mu_p(6);

    mu_p(0) = mu(0) + mu(3) * dt + u(0);
    mu_p(1) = mu(1) + mu(4) * dt + u(1);
    mu_p(2) = mu(2) + mu(5) * dt + u(2);
    mu_p(3) = mu(3)              + u(3);
    mu_p(4) = mu(4)              + u(4);
    mu_p(5) = mu(5)              + u(5);

    return mu_p;
}

Eigen::MatrixXd ekf_attitude_G(
    Eigen::VectorXd &mu,
    Eigen::VectorXd &input,
    float dt
)
{
    // g function linear as currently written
    return Eigen::MatrixXd::Identity(6, 6);
}

Eigen::VectorXd ekf_attitude_h(Eigen::VectorXd &mu_p, float dt)
{
    Eigen::VectorXd y(9);

    // accel (x, y, z) - m/s^2
    y(3) = atan2(mu_p(3), sqrt(pow(mu_p(4), 2) + pow(mu_p(5), 2)));
    y(4) = atan2(mu_p(4), sqrt(pow(mu_p(3), 2) + pow(mu_p(5), 2)));
    y(5) = atan2(mu_p(5), sqrt(pow(mu_p(3), 2) + pow(mu_p(4), 2)));

    // gyro (x, y, z) - rad/s
    y(6) = mu_p(0);
    y(7) = mu_p(1);
    y(8) = mu_p(2);

    // mag (x, y, z) - units?
    y(0) = mu_p(0);
    y(1) = mu_p(1);
    y(2) = mu_p(2);

    return y;
}

Eigen::MatrixXd ekf_attitude_H(Eigen::VectorXd & mu_p, float dt)
{
    Eigen::MatrixXd H(9, 6);

    // fill H with zero
    H.topLeftCorner(9, 6) = Eigen::MatrixXd::Zero(9, 6);

    // magnometer differential results in identity matrix
    H.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);

    // gyro differential also results in identity matrix
    H.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);

    // linearize accelerometer measurments
    float a = sqrt(pow(mu_p[4], 2) + pow(mu_p[5], 2));
    float b = sqrt(pow(mu_p[3], 2) + pow(mu_p[5], 2));
    float c = sqrt(pow(mu_p[3], 2) + pow(mu_p[4], 2));
    float L = pow(mu_p[3], 2) + pow(mu_p[4], 2) + pow(mu_p[5], 2);

    // add each entry into the H matrix
    H(3, 3) = a / L;
    H(3, 4) = -mu_p[3] * mu_p[4] / (a * L);
    H(3, 5) = -mu_p[3] * mu_p[5] / (a * L);

    H(4, 3) = -mu_p[3] * mu_p[4] / (b * L);
    H(4, 4) = b / L;
    H(4, 5) = -mu_p[4] * mu_p[5] / (b * L);

    H(5, 3) = -mu_p[3] * mu_p[5] / (c * L);
    H(5, 4) = -mu_p[4] * mu_p[5] / (c * L);
    H(5, 5) = c / L;

    return H;
}

void ekf_attitude_estimator_initialize(struct ekf *e, Eigen::VectorXd mu)
{
    Eigen::MatrixXd S;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd I;

    // initialize values
    S = Eigen::MatrixXd::Identity(mu.size(), mu.size());
    R = Eigen::MatrixXd::Identity(mu.size(), mu.size());
    I = Eigen::MatrixXd::Identity(mu.size(), mu.size());
    Q = Eigen::MatrixXd::Zero(9, 9);

    // best guess for the R values process noise
    R(0, 0) = 0.05;
    R(1, 1) = 0.05;
    R(2, 2) = 0.05;

    R(3, 3) = 0.0005;
    R(4, 4) = 0.0005;
    R(5, 5) = 0.0005;

    // magnometer measurement noise
    Q(0, 0) = 20.0;
    Q(1, 1) = 20.0;
    Q(2, 2) = 20.0;

    // accelerometer measurement noise
    Q(3, 3) = 0.2;
    Q(4, 4) = 0.2;
    Q(5, 5) = 0.2;

    // gyro measurement noise
    Q(6, 6) = 0.02;
    Q(7, 7) = 0.02;
    Q(8, 8) = 0.02;

    // assign inital values to ekf
    e->mu = mu;
    e->S  = S;
    e->R  = R;
    e->I  = I;
    e->Q  = Q;

    // assign g, G, h, and H functions
    e->g_function = ekf_attitude_g;
    e->G_function = ekf_attitude_G;
    e->h_function = ekf_attitude_h;
    e->H_function = ekf_attitude_H;
}
