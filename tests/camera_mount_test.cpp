#include "awesomo/munit.h"
#include "awesomo/camera_mount.hpp"


// TESTS
int testCameraMountGetTargetPositionBFrame(void);
int testCameraMountGetTargetPositionBPFrame(void);


static void print_target_relative_to_quad(Eigen::Vector3d &target)
{
    printf("target position (relative to quad): ");
    printf("%f, ", target(0));
    printf("%f, ", target(1));
    printf("%f\n", target(2));
    printf("\n");
}

int testCameraMountGetTargetPositionBFrame(void)
{
    CameraMount mount;
    Eigen::Vector3d target;
    Eigen::Vector3d target_BF;

    float roll = 0.0;
    float pitch = deg2rad(-90);
    float yaw = 0.0;
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;

    mount = CameraMount(roll, pitch, yaw, dx, dy, dz);

    // target is front of camera
    target << 1.0, 0.0, 0.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), 0.0) == 0);
    mu_check(fltcmp(target_BF(1), 0.0) == 0);
    mu_check(fltcmp(target_BF(2), 1.0) == 0);
    printf("front of camera\n");
    print_target_relative_to_quad(target_BF);

    // target left of camera
    target << 0.0, -1.0, 0.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), 0.0) == 0);
    mu_check(fltcmp(target_BF(1), -1.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("left of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is right of camera
    target << 0.0, 1.0, 0.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), 0.0) == 0);
    mu_check(fltcmp(target_BF(1), 1.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("right of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is top of camera
    target << 0.0, 0.0, -1.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), 1.0) == 0);
    mu_check(fltcmp(target_BF(1), 0.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("top of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is bottom of camera
    target << 0.0, 0.0, 1.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), -1.0) == 0);
    mu_check(fltcmp(target_BF(1), 0.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("bottom of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is top-left of camera
    target << 0.0, -1.0, -1.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), 1.0) == 0);
    mu_check(fltcmp(target_BF(1), -1.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("top-left of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is top-right of camera
    target << 0.0, 1.0, -1.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), 1.0) == 0);
    mu_check(fltcmp(target_BF(1), 1.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("top-right of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is bottom-left of camera
    target << 0.0, -1.0, 1.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), -1.0) == 0);
    mu_check(fltcmp(target_BF(1), -1.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("bottom-left of camera\n");
    print_target_relative_to_quad(target_BF);

    // target is bottom-right of camera
    target << 0.0, 1.0, 1.0;
    target_BF = mount.getTargetPositionBFrame(target);
    mu_check(fltcmp(target_BF(0), -1.0) == 0);
    mu_check(fltcmp(target_BF(1), 1.0) == 0);
    mu_check(fltcmp(target_BF(2), 0.0) == 0);
    printf("bottom-right of camera\n");
    print_target_relative_to_quad(target_BF);

    return 0;
}

int testCameraMountGetTargetPositionBPFrame(void)
{
    CameraMount mount;
    Eigen::Vector3d target;
    Eigen::Quaterniond imu;
    Eigen::Vector3d target_BPF;

    float roll = 0.0;
    float pitch = deg2rad(-90);
    float yaw = 0.0;
    float dx = 0.0;
    float dy = 0.0;
    float dz = 0.0;

    // setup
    mount = CameraMount(roll, pitch, yaw, dx, dy, dz);

    // target is front of camera
    target << 10.0, 0.0, 0.0;
    euler2Quaternion(0, deg2rad(-10), 0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    // mu_check(fltcmp(target_BPF(0), 0.0) == 0);
    // mu_check(fltcmp(target_BPF(1), 0.0) == 0);
    // mu_check(fltcmp(target_BPF(2), 1.0) == 0);
    printf("front of camera\n");
    print_target_relative_to_quad(target_BPF);

    // target left of camera
    target << 0.0, -10.0, 0.0;
    euler2Quaternion(0, deg2rad(-10), 0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    // mu_check(fltcmp(target_BPF(0), 0.0) == 0);
    // mu_check(fltcmp(target_BPF(1), 0.0) == 0);
    // mu_check(fltcmp(target_BPF(2), 1.0) == 0);
    printf("left of camera\n");
    print_target_relative_to_quad(target_BPF);

    return 0;
}

void test_suite(void)
{
    mu_add_test(testCameraMountGetTargetPositionBFrame);
    mu_add_test(testCameraMountGetTargetPositionBPFrame);
}

mu_run_tests(test_suite)
