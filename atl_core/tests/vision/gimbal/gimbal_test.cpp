#include "atl_core/atl_test.hpp"
#include "atl_core/vision/gimbal/gimbal.hpp"

#define TEST_CONFIG "tests/configs/gimbal/config.yaml"

namespace atl {

static void print_target_relative_to_body_frame(Vec3 &target) {
  std::cout << "target position (relative to quad in body frame): ";
  std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

static void print_target_relative_to_body_planar_frame(Vec3 &target) {
  std::cout << "target position (relative to quad in body planar frame): ";
  std::cout << std::fixed << std::setprecision(2) << target(0) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(1) << " ";
  std::cout << std::fixed << std::setprecision(2) << target(2) << std::endl;
}

TEST(Gimbal, constructor) {
  Gimbal gimbal;

  ASSERT_FALSE(gimbal.configured);

  ASSERT_FALSE(gimbal.sbgc.configured);
}

// TEST(Gimbal, configure) {
//   Gimbal gimbal;
//
//   gimbal.configure(TEST_CONFIG);
//
//   ASSERT_TRUE(gimbal.configured);
//
//   ASSERT_EQ("/dev/ttyUSB0", gimbal.sbgc.port);
//
//   ASSERT_FLOAT_EQ(1.0, gimbal.camera_offset.position(0));
//   ASSERT_FLOAT_EQ(2.0, gimbal.camera_offset.position(1));
//   ASSERT_FLOAT_EQ(3.0, gimbal.camera_offset.position(2));
//
//   ASSERT_FLOAT_EQ(deg2rad(1.0), gimbal.limits[0]);
//   ASSERT_FLOAT_EQ(deg2rad(2.0), gimbal.limits[1]);
//   ASSERT_FLOAT_EQ(deg2rad(3.0), gimbal.limits[2]);
//   ASSERT_FLOAT_EQ(deg2rad(4.0), gimbal.limits[3]);
//   ASSERT_FLOAT_EQ(deg2rad(5.0), gimbal.limits[4]);
//   ASSERT_FLOAT_EQ(deg2rad(6.0), gimbal.limits[5]);
// }

TEST(Gimbal, getTargetInBF) {
  Gimbal gimbal;
  Vec3 target_cf;
  Vec3 target_bf;

  double roll = 0.0;
  double pitch = deg2rad(90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);

  // target is front of camera
  target_cf << 0.0, 0.0, 1.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is front of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(0.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(2), 0.0000001);

  // target left of camera
  target_cf << -1.0, 0.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is left of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(0.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is right of camera
  target_cf << 1.0, 0.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is right of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(0.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is top of camera
  target_cf << 0.0, -1.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is top of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is bottom of camera
  target_cf << 0.0, 1.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is bottom of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(-1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is top-left of camera
  target_cf << -1.0, -1.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is top-left of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is top-right of camera
  target_cf << 1.0, -1.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is top-right of camera]\t\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is bottom-left of camera
  target_cf << -1.0, 1.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is bottom-left of camera]\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(-1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);

  // target is bottom-right of camera
  target_cf << 1.0, 1.0, 0.0;
  target_bf = gimbal.getTargetInBF(gimbal.camera_offset, target_cf);
  std::cout << "[target is bottom-right of camera]\t";
  print_target_relative_to_body_frame(target_bf);
  ASSERT_NEAR(-1.0, target_bf(0), 0.0000001);
  ASSERT_NEAR(-1.0, target_bf(1), 0.0000001);
  ASSERT_NEAR(0.0, target_bf(2), 0.0000001);
}

TEST(Gimbal, getTargetInBPF) {
  Gimbal gimbal;
  Quaternion joint;
  Vec3 euler;
  Vec3 target_cf;
  Vec3 target_bpf;

  double roll = 0.0;
  double pitch = deg2rad(90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  target_cf << 0.0, 0.0, 10.0;  // let tag be directly infront of camera
  gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);
  std::cout << "target is directly infront of camera: ";
  std::cout << "[" << target_cf.transpose() << "]" << std::endl;

  // pitch forwards
  euler << 0.0, deg2rad(10), 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor pitch forwards]\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) < 0.0);
  ASSERT_NEAR(0.0, target_bpf(1), 0.000000001);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // pitch backwards
  euler << 0.0, deg2rad(-10), 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor pitch backwards]\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) > 0.0);
  ASSERT_NEAR(0.0, target_bpf(1), 0.000000001);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // roll left
  euler << deg2rad(-10), 0.0, 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor roll left]\t\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_NEAR(0.0, target_bpf(0), 0.0000000001);
  ASSERT_TRUE(target_bpf(1) < 0.0);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // roll right
  euler << deg2rad(10), 0.0, 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor roll right]\t\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_NEAR(0.0, target_bpf(0), 0.0000000001);
  ASSERT_TRUE(target_bpf(1) > 0.0);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // pitch forward, roll left
  euler << deg2rad(-10), deg2rad(10), 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor pitch forward, roll left]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) < 0.0);
  ASSERT_TRUE(target_bpf(1) < 0.0);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // pitch forward, roll right
  euler << deg2rad(10), deg2rad(10), 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor pitch forward, roll right]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) < 0.0);
  ASSERT_TRUE(target_bpf(1) > 0.0);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // pitch backwards, roll left
  euler << deg2rad(-10), deg2rad(-10), 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor pitch backwards, roll left]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) > 0.0);
  ASSERT_TRUE(target_bpf(1) < 0.0);
  ASSERT_TRUE(target_bpf(2) < 0.0);

  // pitch backwards, roll right
  euler << deg2rad(10), deg2rad(-10), 0.0;
  euler2quat(euler, 321, joint);
  target_bpf = gimbal.getTargetInBPF(gimbal.camera_offset, target_cf, joint);
  std::cout << "[quadrotor pitch backwards, roll right]\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  ASSERT_TRUE(target_bpf(0) > 0.0);
  ASSERT_TRUE(target_bpf(1) > 0.0);
  ASSERT_TRUE(target_bpf(2) < 0.0);
}

TEST(Gimbal, getTargetInIF) {
  Quaternion frame;
  Vec3 euler, target_bpf, gimbal_position, target_if;

  // setup
  target_bpf << 1.0, 2.0, 0.0;
  gimbal_position << 0.0, 1.0, 2.0;

  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  euler2quat(euler, 321, frame);

  target_if = Gimbal::getTargetInIF(target_bpf, gimbal_position, frame);
  std::cout << "target if: " << target_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-2.0, target_if(0));
  ASSERT_FLOAT_EQ(2.0, target_if(1));
  ASSERT_FLOAT_EQ(2.0, target_if(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  euler2quat(euler, 321, frame);

  target_if = Gimbal::getTargetInIF(target_bpf, gimbal_position, frame);
  std::cout << "target if: " << target_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_if(0));
  ASSERT_FLOAT_EQ(-1.0, target_if(1));
  ASSERT_FLOAT_EQ(2.0, target_if(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  euler2quat(euler, 321, frame);

  target_if = Gimbal::getTargetInIF(target_bpf, gimbal_position, frame);
  std::cout << "target if: " << target_if.transpose() << std::endl;

  ASSERT_NEAR(2.0, target_if(0), 0.001);
  ASSERT_NEAR(0.0, target_if(1), 0.001);
  ASSERT_NEAR(2.0, target_if(2), 0.001);

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  euler2quat(euler, 321, frame);

  target_if = Gimbal::getTargetInIF(target_bpf, gimbal_position, frame);
  std::cout << "target if: " << target_if.transpose() << std::endl;

  ASSERT_NEAR(1.0, target_if(0), 0.001);
  ASSERT_NEAR(3.0, target_if(1), 0.001);
  ASSERT_NEAR(2.0, target_if(2), 0.001);
}

TEST(Gimbal, trackTarget) {
  Gimbal gimbal;
  Quaternion imu;
  Vec3 euler;
  Vec3 target_cf;
  Vec3 target_bpf;

  double roll = 0.0;
  double pitch = deg2rad(90);
  double yaw = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  target_cf << 0.0, 0.0, 10.0;  // let tag be directly infront of camera
  gimbal.camera_offset = Pose(roll, pitch, yaw, dx, dy, dz);
  std::cout << "target is directly infront of camera: ";
  std::cout << "[" << target_cf.transpose() << "]" << std::endl;

  // pitch forwards
  euler << 0.0, deg2rad(10), 0.0;
  euler2quat(euler, 321, imu);
  target_bpf = Gimbal::getTargetInBPF(gimbal.camera_offset, target_cf, imu);
  std::cout << "[quadrotor pitch forwards]\t\t";
  print_target_relative_to_body_planar_frame(target_bpf);
  gimbal.trackTarget(target_bpf);
  std::cout << gimbal.setpoints.transpose() << std::endl;
}

}  // end of atl namespace
