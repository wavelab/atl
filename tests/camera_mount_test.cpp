#include "awesomo/munit.h"
#include "awesomo/camera_mount.hpp"


// TESTS
int testCameraMountContstuctor(void);
int testCameraMountToBodyFrame(void);
int testCameraMountToBPFrame(void);


int testCameraConfigAtimToBodyFrame(void)
{

    CameraMount mount;
    Eigen::Vector3d target_position;
    Eigen::Vector3d target_position_BF;

    float roll = 0.0;
    float pitch = -1 * M_PI / 2;
    float yaw = 0.0;
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;

    mount = CameraMount(roll, pitch, yaw, dx, dy, dz );

    target_position << 1.0, 0, 0;

    target_position_BF = mount.getTargetPositionBFrame(target_position);
    // std::cout << target_position_BF(2) << std::endl;
    mu_check(fltcmp(target_position_BF(2), 1) == 0);


    // to do: test all possible mounts

    // target_position.x = 0.0;
    // target_position.y = 0.0;
    // target_position.z = 1.0;
    //
    // mount.getAtimTargetPositionBodyFrame(
    //     target_position,
    //     target_position_BF
    // );
    //
    // std::cout << target_position_BF.x << "\t"
    //           << target_position_BF.y << "\t"
    //           << target_position_BF.z << "\t"
    //           << std::endl;
    return 0;
}

void test_suite(void)
{
    mu_add_test(testCameraConfigAtimToBodyFrame);
}

mu_run_tests(test_suite)
