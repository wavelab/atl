#include "munit.h"
#include "ekf.hpp"
#include <tgmath.h>

// TESTS
int testPredictionUpdate(void);
int testMeasurementUpdate(void);


// ekf g_function
Eigen::VectorXd g_function(Eigen::VectorXd & mu, Eigen::VectorXd & input, double dt){
    // work with only roll, pitch, yaw and first order derivatives
    // mu[0] = roll
    // mu[1] = pitch
    // mu[2] = yaw
    // mu[3] = roll_dot
    // mu[4] = pitch_dot
    // mu[5] = yaw_dot

    Eigen::VectorXd mu_bar(6);
    mu_bar(0)   = mu(0) + mu(3)*dt + input(0);
    mu_bar(1)   = mu(1) + mu(4)*dt + input(1);
    mu_bar(2)   = mu(2) + mu(5)*dt + input(2);
    mu_bar(3)   = mu(3)            + input(3);
    mu_bar(4)   = mu(4)            + input(4);
    mu_bar(5)   = mu(5)            + input(5);

    return mu_bar;
}

Eigen::MatrixXd G_function(Eigen::VectorXd & mu, Eigen::VectorXd &input, double dt){
    //this g function linear as currently written
    return Eigen::MatrixXd::Identity(6,6);
}

Eigen::VectorXd h_function(Eigen::VectorXd & m, double dt){
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

Eigen::MatrixXd H_function (Eigen::VectorXd & mu_p, double dt){
    Eigen::MatrixXd H(9, 6);
    H.topLeftCorner(9, 6) = Eigen::MatrixXd::Zero(9, 6);

    // magnometer differential results in identity matrix
    H.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3, 3);

    // gyro differential also results in identity matrix
    H.bottomLeftCorner(3,3) = Eigen::MatrixXd::Identity(3, 3);

    // linearize accelerometer measurments
    double a = sqrt(mu_p[4] * mu_p[4] + mu_p[5] * mu_p[5]);
    double b = sqrt(mu_p[3] * mu_p[3] + mu_p[5] * mu_p[5]);
    double c = sqrt(mu_p[3] * mu_p[3] + mu_p[4] * mu_p[4]);

    double L = mu_p[3] * mu_p[3] + mu_p[4] * mu_p[4] + mu_p[5] * mu_p[5];

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


int testPredictionUpdate(void)
{
    ekf prediction_ekf;

    double dt;
    dt = 0.1;

    Eigen::VectorXd mu(6);
    mu << 1, 1, 1, 1, 1, 1;

    Eigen::MatrixXd S;
    S = Eigen::MatrixXd::Identity(mu.size(), mu.size());

    Eigen::MatrixXd R;
    S = Eigen::MatrixXd::Identity(mu.size(), mu.size());

    Eigen::VectorXd u_test(6);
    u_test << 1, 1, 1, 1, 1, 1;

    prediction_ekf.mu = mu;
    prediction_ekf.S  = S;
    prediction_ekf.dt = dt;
    prediction_ekf.R  = R;

    ekf_prediction_update( &prediction_ekf, &g_function, &G_function, u_test);
	return 0;
}

int testMeasurementUpdate(void)
{
    ekf measurement_ekf;

    double dt;
    dt = 0.1;

    Eigen::VectorXd mu(6);
    mu << 1, 1, 1, 1, 1, 1;

    Eigen::VectorXd mu_p(6);
    mu_p << 1, 1, 1, 1, 1, 1;

    Eigen::MatrixXd S;
    S = Eigen::MatrixXd::Identity(mu_p.size(), mu_p.size());

    Eigen::VectorXd y_test(9);
    y_test << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    Eigen::MatrixXd Q;
    Q = Eigen::MatrixXd::Identity(y_test.size(), y_test.size());

    measurement_ekf.mu = mu;
    measurement_ekf.mu_p = mu_p;
    measurement_ekf.Q = Q;
    measurement_ekf.S_p = S;
    measurement_ekf.dt = dt;

    ekf_measurement_update( &measurement_ekf, &h_function, &H_function, y_test);
	return 0;
}


void testSuite(void)
{
    mu_add_test(testPredictionUpdate);
    mu_add_test(testMeasurementUpdate);
}

mu_run_tests(testSuite)
