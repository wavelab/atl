#include "awesomo/munit.h"
#include "awesomo/ros/quadrotor_node.hpp"


// TESTS
int testQuadrotorPoseCallback(void);


int testQuadrotorPoseCallback(void)
{
    Quadrotor q;


    return 0;
}



void test_suite(void)
{
    mu_add_test(testQuadrotorPoseCallback);
}

mu_run_tests(test_suite)
