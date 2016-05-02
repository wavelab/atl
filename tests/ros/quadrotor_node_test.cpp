#include "awesomo/munit.h"
#include "awesomo/ros/quadrotor.hpp"


// TESTS
int testQuadrotorPositionControllerCalculate(void);


int testQuadrotorPositionControllerCalculate(void)
{
    Quadrotor q;
    ros::Time last_request;

    last_request = ros::Time::now();
    q.positionControllerCalculate(0, 0, 0, last_request);

    return 0;
}


void test_suite(void)
{
    mu_add_test(testQuadrotorPositionControllerCalculate);
}

mu_run_tests(test_suite)
