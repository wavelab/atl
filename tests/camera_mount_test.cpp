#include "awesomo/munit.h"
#include "awesomo/camera_mount.hpp"

// CONFIGS
#define CAMERA_MOUNT_CONFIG "configs/camera_mount/config.yaml"

// TESTS
int testCameraMountGetTargetPositionBFrame(void);
int testCameraMountGetTargetPositionBPFrame(void);


static void print_target_relative_to_quad(Eigen::Vector3d &target)
{
    printf("target position (relative to quad in body planar frame): ");
    printf("%f, ", target(0));
    printf("%f, ", target(1));
    printf("%f\n", target(2));
    printf("\n");
}

CameraMount *testSetup(void)
{
    CameraMount *cam_mount;
    std::map<std::string, std::string> configs;
    configs["camera_mount"] = CAMERA_MOUNT_CONFIG;
    cam_mount = new CameraMount(configs);

    return cam_mount;
}

int testCameraMount(void)
{
    CameraMount *cam_mount;
    // setup
    cam_mount = testSetup();

    std::cout << cam_mount->gimbal_limits.pitch_limits << std::endl;
    // Todo: make a sample config and check values
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
    target << 10.0, 0.0, 0.0;  // let tag be directly infront of camera
    mount = CameraMount(roll, pitch, yaw, dx, dy, dz);

    // pitch forwards
    euler2Quaternion(0.0, deg2rad(-10), 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(target_BPF(0) < 0.0);
    mu_check(fltcmp(target_BPF(1), 0.0) == 0);
    mu_check(target_BPF(2) < 10.0);
    printf("pitch forwards\n");
    print_target_relative_to_quad(target_BPF);

    // pitch backwards
    euler2Quaternion(0.0, deg2rad(10), 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(target_BPF(0) > 0.0);
    mu_check(fltcmp(target_BPF(1), 0.0) == 0);
    mu_check(target_BPF(2) < 10.0);
    printf("pitch backwards\n");
    print_target_relative_to_quad(target_BPF);

    // roll left
    euler2Quaternion(deg2rad(-10), 0.0, 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(fltcmp(target_BPF(0), 0.0) == 0);
    mu_check(target_BPF(1) > 0.0);
    mu_check(target_BPF(2) < 10.0);
    printf("roll left\n");
    print_target_relative_to_quad(target_BPF);

    // roll right
	euler2Quaternion(deg2rad(10), 0.0, 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(fltcmp(target_BPF(0), 0.0) == 0);
    mu_check(target_BPF(1) < 0.0);
    mu_check(target_BPF(2) < 10.0);
    printf("roll right\n");
    print_target_relative_to_quad(target_BPF);

    // pitch forward, roll left
	euler2Quaternion(deg2rad(-10), deg2rad(-10), 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(target_BPF(0) < 0.0);
    mu_check(target_BPF(1) > 0.0);
    mu_check(target_BPF(2) < 10.0);
    printf("pitch forward, roll left\n");
    print_target_relative_to_quad(target_BPF);

    // pitch forward, roll right
	euler2Quaternion(deg2rad(10), deg2rad(-10), 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(target_BPF(0) < 0.0);
    mu_check(target_BPF(1) < 0.0);
    mu_check(target_BPF(2) < 10.0);
    printf("pitch forward, roll right\n");
    print_target_relative_to_quad(target_BPF);

    // pitch backwards, roll left
	euler2Quaternion(deg2rad(-10), deg2rad(10), 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(target_BPF(0) > 0.0);
    mu_check(target_BPF(1) > 0.0);
    mu_check(target_BPF(2) < 10.0);
    printf("pitch backwards, roll left\n");
    print_target_relative_to_quad(target_BPF);

    // pitch backwards, roll right
	euler2Quaternion(deg2rad(10), deg2rad(10), 0.0, imu);
    target_BPF = mount.getTargetPositionBPFrame(target, imu);
    mu_check(target_BPF(0) > 0.0);
    mu_check(target_BPF(1) < 0.0);
    mu_check(target_BPF(2) < 10.0);
    printf("pitch backwards, roll right\n");
    print_target_relative_to_quad(target_BPF);

    return 0;
}

void test_suite(void)
{
    mu_add_test(testCameraMount);
    mu_add_test(testCameraMountGetTargetPositionBFrame);
    mu_add_test(testCameraMountGetTargetPositionBPFrame);
}

mu_run_tests(test_suite)
