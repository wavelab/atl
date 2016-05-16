#include "munit.h"
#include "ekf.hpp"
#include <tgmath.h>
#include <stdio.h>
#include <stdlib.h>

// GLOBAL VARIRABLES
struct ekf estimator;

// TESTS
int testPredictionUpdate(void);
int testMeasurementUpdate(void);


void ekf_new(struct ekf *att_ekf)
{
    Eigen::VectorXd mu(6);
    mu <<
        1,  // roll
        1,  // pitch
        1,  // yaw
        1,  // roll vel
        1,  // pitch vel
        1;  // yaw vel
    ekf_attitude_estimator_initialize(att_ekf, mu);
}

int testPredictionUpdate(void)
{
    Eigen::VectorXd u_test(6);
    float dt;

    u_test << 1, 1, 1, 1, 1, 1;
    dt = 0.01;
    ekf_prediction_update(&estimator, u_test, dt);
	return 0;
}

int testMeasurementUpdate(void)
{
    Eigen::VectorXd y_test(9);
    float dt;

    y_test <<
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9;
    dt = 0.01;

    ekf_measurement_update(&estimator, y_test, dt);
	return 0;
}

void testSuite(void)
{
    ekf_new(&estimator);
    mu_add_test(testPredictionUpdate);
    mu_add_test(testMeasurementUpdate);
}

mu_run_tests(testSuite)
