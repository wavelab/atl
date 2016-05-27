#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
void createCamConfig(CameraMountConfig &config);
int testCameraConfig(void);
int testCameraConfigInitialize(void);
int testCameraConfigInitializeMirror(void);
int testCameraConfigApplyRBTtoPosition(void);


void createCamConfig(CameraMountConfig &config)
{
    config.initialize(M_PI/2, 0, 0, 1, 0.5, -2);
    config.initializeMirrorMtx(-1, 1, 1);
}

int testCameraConfig(void)
{
   CameraMountConfig config;
   return 0;
}

int testCameraConfigInitialize(void)
{
    CameraMountConfig config;
    config.initialize(M_PI/2, M_PI, M_PI, 1, 0.5, -2);
    config.initializeMirrorMtx(-1, 1, 1);

    // std::cout << config.camRotation << std::endl;

    mu_check(fltcmp(config.camRotation(0, 0), 1) == 0);
    mu_check(fltcmp(config.camRotation(1, 1), 0) == 0);
    mu_check(fltcmp(config.camRotation(2, 2), 0) == 0);
    mu_check(fltcmp(config.camRotation(1, 2), 1) == 0);
    mu_check(fltcmp(config.camRotation(2, 1), -1) == 0);
    return 0;
}

int testCameraConfigInitializeMirror(void)
{
    CameraMountConfig config;
    config.initializeMirrorMtx(-1, 1, 1);

    // std::cout << config.camMirroring;
    mu_check(fltcmp(config.camMirroring(0, 0), -1) == 0);
    mu_check(fltcmp(config.camMirroring(1, 1), 1) == 0);
    mu_check(fltcmp(config.camMirroring(2, 2), 1) == 0);

    return 0;
}

int testCameraConfigApplyRBTtoPosition(void)
{
    CameraMountConfig config;
    config.initialize(M_PI/2, M_PI/2, 0, -2, 0, 0);
    // config.initializeMirrorMtx(-1, 1, 1);

    std::cout << config.camRotation << std::endl;

    Position position1;
    position1.x = 0;
    position1.y = 1;
    position1.z = 0;

    config.applyRBTtoPosition(position1);
    std::cout << position1.x << "\n"
              << position1.y << "\n"
              << position1.z << "\n"
              << std::endl;
    return 0;
}

void test_suite(void)
{
    mu_add_test(testCameraConfigInitialize);
    mu_add_test(testCameraConfigInitializeMirror);
    mu_add_test(testCameraConfigApplyRBTtoPosition);
}

mu_run_tests(test_suite)
