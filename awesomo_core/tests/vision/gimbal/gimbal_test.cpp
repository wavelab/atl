#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/vision/gimbal/gimbal.hpp"

#define TEST_CONFIG "tests/configs/gimbal/config.yaml"

static void print_target_relative_to_body_frame(awesomo::Vec3 &target)
{
  std::cout << "target position (relative to quad in body frame): ";
  std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

static void print_target_relative_to_body_planar_frame(awesomo::Vec3 &target)
{
  std::cout << "target position (relative to quad in body planar frame): ";
  std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

TEST(Gimbal, constructor) {
  awesomo::Gimbal gimbal;

  ASSERT_FALSE(gimbal.configured);
  ASSERT_FALSE(gimbal.sim_mode);

  ASSERT_FALSE(gimbal.sbgc.configured);
}

TEST(Gimbal, configure) {
  awesomo::Gimbal gimbal;

  gimbal.configure(TEST_CONFIG);

  ASSERT_TRUE(gimbal.configured);
  ASSERT_FALSE(gimbal.sim_mode);

  ASSERT_EQ("/dev/ttyUSB0", gimbal.sbgc.port);

  ASSERT_FLOAT_EQ(1.0, gimbal.camera_offset.position(0));
  ASSERT_FLOAT_EQ(2.0, gimbal.camera_offset.position(1));
  ASSERT_FLOAT_EQ(3.0, gimbal.camera_offset.position(2));

  ASSERT_FLOAT_EQ(awesomo::deg2rad(1.0), gimbal.limits[0]);
  ASSERT_FLOAT_EQ(awesomo::deg2rad(2.0), gimbal.limits[1]);
  ASSERT_FLOAT_EQ(awesomo::deg2rad(3.0), gimbal.limits[2]);
  ASSERT_FLOAT_EQ(awesomo::deg2rad(4.0), gimbal.limits[3]);
  ASSERT_FLOAT_EQ(awesomo::deg2rad(5.0), gimbal.limits[4]);
  ASSERT_FLOAT_EQ(awesomo::deg2rad(6.0), gimbal.limits[5]);
}

TEST(Gimbal, getTargetPositionInBodyFrame) {
  awesomo::Gimbal gimbal;
  awesomo::Vec3 target_cf;
  awesomo::Vec3 target_bf;

  double roll = 0.0;
  double pitch = awesomo::deg2rad(-90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  gimbal.camera_offset = awesomo::Pose(roll, pitch, yaw, dx, dy, dz);

  // target is front of camera
  target_cf << 1.0, 0.0, 0.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is front of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(0.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(2), 0.0000001);

  // target left of camera
  target_cf << 0.0, -1.0, 0.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is left of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(0.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is right of camera
  target_cf << 0.0, 1.0, 0.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is right of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(0.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is top of camera
  target_cf << 0.0, 0.0, -1.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is top of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is bottom of camera
  target_cf << 0.0, 0.0, 1.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is bottom of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(-1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is top-left of camera
  target_cf << 0.0, -1.0, -1.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is top-left of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is top-right of camera
  target_cf << 0.0, 1.0, -1.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is top-right of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is bottom-left of camera
  target_cf << 0.0, -1.0, 1.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is bottom-left of camera]\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(-1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is bottom-right of camera
  target_cf << 0.0, 1.0, 1.0;
  target_bf = gimbal.getTargetPositionInBodyFrame(target_cf);
  std::cout << "[target is bottom-right of camera]\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(-1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);
}

TEST(Gimbal, getTargetPositionInBodyPlanarFrame) {
  awesomo::Gimbal gimbal;
  awesomo::Quaternion imu;
  awesomo::Vec3 target_cf;
  awesomo::Vec3 target_bpf;

  double roll = 0.0;
  double pitch = awesomo::deg2rad(-90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  target_cf << 10.0, 0.0, 0.0;  // let tag be directly infront of camera
  gimbal.camera_offset = awesomo::Pose(roll, pitch, yaw, dx, dy, dz);
  std::cout << "target is directly infront of camera: ";
  std::cout << "[" << target_cf.transpose() << "]" << std::endl;

  // pitch forwards
  awesomo::euler2Quaternion(0.0, awesomo::deg2rad(-10), 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor pitch forwards]\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) < 0.0);
  ASSERT_NEAR(0.0, target_bpf(1), 0.000000001);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // pitch backwards
  awesomo::euler2Quaternion(0.0, awesomo::deg2rad(10), 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor pitch backwards]\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) > 0.0);
  ASSERT_NEAR(0.0, target_bpf(1), 0.000000001);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // roll left
  awesomo::euler2Quaternion(awesomo::deg2rad(-10), 0.0, 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor roll left]\t\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_NEAR(0.0, target_bpf(0), 0.0000000001);
  ASSERT_TRUE(target_bpf(1) > 0.0);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // roll right
  awesomo::euler2Quaternion(awesomo::deg2rad(10), 0.0, 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor roll right]\t\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_NEAR(0.0, target_bpf(0), 0.0000000001);
  ASSERT_TRUE(target_bpf(1) < 0.0);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // pitch forward, roll left
  awesomo::euler2Quaternion(awesomo::deg2rad(-10), awesomo::deg2rad(-10), 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor pitch forward, roll left]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) < 0.0);
  ASSERT_TRUE(target_bpf(1) > 0.0);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // pitch forward, roll right
  awesomo::euler2Quaternion(awesomo::deg2rad(10), awesomo::deg2rad(-10), 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor pitch forward, roll right]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) < 0.0);
  ASSERT_TRUE(target_bpf(1) < 0.0);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // pitch backwards, roll left
  awesomo::euler2Quaternion(awesomo::deg2rad(-10), awesomo::deg2rad(10), 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor pitch backwards, roll left]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) > 0.0);
  ASSERT_TRUE(target_bpf(1) > 0.0);
  ASSERT_TRUE(target_bpf(2) < 10.0);

  // pitch backwards, roll right
  awesomo::euler2Quaternion(awesomo::deg2rad(10), awesomo::deg2rad(10), 0.0, imu);
  target_bpf = gimbal.getTargetPositionInBodyPlanarFrame(target_cf, imu);
  std::cout << "[quadrotor pitch backwards, roll right]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) > 0.0);
  ASSERT_TRUE(target_bpf(1) < 0.0);
  ASSERT_TRUE(target_bpf(2) < 10.0);
}
