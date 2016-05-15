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

Eigen::VectorXd att_g_function(Eigen::VectorXd &mu, Eigen::VectorXd &u,
                               float dt){

    // work with only roll, pitch, yaw and first order derivatives
    // mu[0] = roll
    // mu[1] = pitch
    // mu[2] = yaw
    // mu[3] = roll_dot
    // mu[4] = pitch_dot
    // mu[5] = yaw_dot

    Eigen::VectorXd mu_p(6);

    mu_p(0)   = mu(0) + mu(3)*dt + u(0);
    mu_p(1)   = mu(1) + mu(4)*dt + u(1);
    mu_p(2)   = mu(2) + mu(5)*dt + u(2);
    mu_p(3)   = mu(3)            + u(3);
    mu_p(4)   = mu(4)            + u(4);
    mu_p(5)   = mu(5)            + u(5);

    return mu_p;
}

Eigen::MatrixXd att_G_function(Eigen::VectorXd & mu, Eigen::VectorXd &input,
        float dt){
    //this g function linear as currently written
    return Eigen::MatrixXd::Identity(6,6);
}

Eigen::VectorXd att_h_function(Eigen::VectorXd & m, float dt){
    // measurement, m,  vector layout
    // measurement[0] = roll_mag
    // measurement[1] = pitch_mag
    // measurement[2] = yaw_mag
    // measurement[3] = accelerometer_x
    // measurement[4] = accelerometer_y
    // measurement[5] = accelerometer_z
    // measurement[6] = gyro_roll_rate
    // measurement[7] = gyro_pitch_rate
    // measurement[8] = gyro_yaw_rate

    Eigen::VectorXd y(9);
    // gyro
    y(0) = m(0);
    y(1) = m(1);
    y(2) = m(2);

    y(3) = atan2(m(3), sqrt(m(4)*m(4) + m(5)*m(5)));
    y(4) = atan2(m(4), sqrt(m(3)*m(3) + m(5)*m(5)));
    y(5) = atan2(m(5), sqrt(m(3)*m(3) + m(4)*m(4)));

    y(6) = m(0);
    y(7) = m(1);
    y(8) = m(2);

    return y;
}


Eigen::MatrixXd att_H_function (Eigen::VectorXd & mu_p, float dt){
    Eigen::MatrixXd H(9, 6);
    H.topLeftCorner(9, 6) = Eigen::MatrixXd::Zero(9, 6);

    // magnometer differential results in identity matrix
    H.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3, 3);

    // gyro differential also results in identity matrix
    H.bottomLeftCorner(3,3) = Eigen::MatrixXd::Identity(3, 3);

    // linearize accelerometer measurments
    float a = sqrt(mu_p[4] * mu_p[4] + mu_p[5] * mu_p[5]);
    float b = sqrt(mu_p[3] * mu_p[3] + mu_p[5] * mu_p[5]);
    float c = sqrt(mu_p[3] * mu_p[3] + mu_p[4] * mu_p[4]);

    float L = mu_p[3] * mu_p[3] + mu_p[4] * mu_p[4] + mu_p[5] * mu_p[5];

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

void initialize_att_ekf(struct ekf *new_ekf, Eigen::VectorXd mu)
{

    // Initialize values

    Eigen::MatrixXd R;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd I;

    R = Eigen::MatrixXd::Identity(mu.size(), mu.size());
    I = Eigen::MatrixXd::Identity(mu.size(), mu.size());

    // Assign inital values to ekf
    new_ekf->mu = mu;
    new_ekf->R  = R;
    new_ekf->I  = I;
    new_ekf->Q  = Q;

    // Assign g, G, h, and H functions
    new_ekf->g_function = att_g_function;
    new_ekf->G_function = att_G_function;

    new_ekf->h_function = att_h_function;
    new_ekf->H_function = att_H_function;
}



