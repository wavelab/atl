#include "awesomo/munit.h"
#include "awesomo/ros/quadrotor.hpp"


// TESTS
int testQuadrotorPositionControllerCalculate(void);

int testQuadrotorPositionControllerCalculate(void)
{
    Quadrotor q;
    Position p;
    ros::Time last_request;

    last_request = ros::Time::now();
    p.x = 0;
    p.y = 0;
    p.z = 0;
    q.positionControllerCalculate(p, last_request);

    return 0;
}

void test_suite(void)
{
    mu_add_test(testQuadrotorPositionControllerCalculate);
}

mu_run_tests(test_suite)
