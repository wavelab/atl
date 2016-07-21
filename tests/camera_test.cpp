#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
// void createCamConfig(CameraMountRBT &config);
int testCameraConfig(void);
int testCameraConfigInitialize(void);
int testCameraConfigAtimToBodyFrame(void);



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

int testCameraConfigAtimToBodyFrame(void)
{

    cameraMount config;
    Position target_position;
    Position target_position_BF;

    float roll = 0.0;
    float pitch = -1 * M_PI / 2;
    float yaw = 0.0;
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;

    config.initialize(
        roll,
        pitch,
        yaw,
        dx,
        dy,
        dz
    );

    target_position.x = 1.0;
    target_position.y = 0.0;
    target_position.z = 0.0;

    config.getAtimTargetPositionBodyFrame(
        target_position,
        target_position_BF
    );
    std::cout << target_position_BF.z << std::endl;
    mu_check(fltcmp(target_position_BF.z, 1) == 0);


}



void test_suite(void)
{
    mu_add_test(testCameraConfigInitialize);
    mu_add_test(testCameraConfigAtimToBodyFrame);
}

mu_run_tests(test_suite)
