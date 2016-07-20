#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
// void createCamConfig(CameraMountRBT &config);
int testCameraConfig(void);
int testCameraConfigInitialize(void);



int testCameraConfigInitialize(void)
{
    cameraMount config;
    config.initialize(M_PI/2, M_PI, M_PI, 1, 0.5, -2);
    // std::cout << config.camRotation << std::endl;

    mu_check(fltcmp(config.camRotation(0, 0), 1) == 0);
    mu_check(fltcmp(config.camRotation(1, 1), 0) == 0);
    mu_check(fltcmp(config.camRotation(2, 2), 0) == 0);
    return 0;
}



void test_suite(void)
{
    mu_add_test(testCameraConfigInitialize);
}

mu_run_tests(test_suite)
