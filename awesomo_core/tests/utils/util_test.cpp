#include <gtest/gtest.h>

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

// int test_Pose_class(void) {
//   Pose testPose;
//   Eigen::Quaterniond q_test;
//   Eigen::Vector3d position_test;
//   Eigen::Matrix3d rotation_mtx;
//   Eigen::Matrix3d rotation_mtx_test;
//
//   float roll;
//   float pitch;
//   float yaw;
//   float x;
//   float y;
//   float z;
//
//   // check Pose with no arguments (q = identity, position = 0);
//   mu_check(fltcmp(testPose.q.x(), 0) == 0);
//   mu_check(fltcmp(testPose.q.y(), 0) == 0);
//   mu_check(fltcmp(testPose.q.z(), 0) == 0);
//   mu_check(fltcmp(testPose.q.w(), 1) == 0);
//
//   mu_check(fltcmp(testPose.position(0), 0) == 0);
//   mu_check(fltcmp(testPose.position(1), 0) == 0);
//   mu_check(fltcmp(testPose.position(2), 0) == 0);
//
//   // test initalizeing with floats,
//   roll = 0.0;
//   pitch = 0.0;
//   yaw = 0.0;
//   x = 2;
//   y = 3;
//   z = 22;
//
//   testPose = Pose(roll, pitch, yaw, x, y, z);
//   mu_check(fltcmp(testPose.q.x(), 0) == 0);
//   mu_check(fltcmp(testPose.q.y(), 0) == 0);
//   mu_check(fltcmp(testPose.q.z(), 0) == 0);
//   mu_check(fltcmp(testPose.q.w(), 1) == 0);
//
//   mu_check(fltcmp(testPose.position(0), x) == 0);
//   mu_check(fltcmp(testPose.position(1), y) == 0);
//   mu_check(fltcmp(testPose.position(2), z) == 0);
//
//   // test initializing non zero roll, pitch, yaw
//   roll = 10.0;
//   pitch = 15.0;
//   yaw = -90.0;
//
//   euler2Quaternion(roll, pitch, yaw, q_test);
//   testPose = Pose(roll, pitch, yaw, x, y, z);
//
//   mu_check(fltcmp(testPose.q.x(), q_test.x()) == 0);
//   mu_check(fltcmp(testPose.q.y(), q_test.y()) == 0);
//   mu_check(fltcmp(testPose.q.z(), q_test.z()) == 0);
//   mu_check(fltcmp(testPose.q.w(), q_test.w()) == 0);
//
//   mu_check(fltcmp(testPose.position(0), x) == 0);
//   mu_check(fltcmp(testPose.position(1), y) == 0);
//   mu_check(fltcmp(testPose.position(2), z) == 0);
//
//   // test inializing with quaterion and a postion vector
//   testPose = Pose(q_test, Eigen::Vector3d(x, y, z));
//   mu_check(fltcmp(testPose.q.x(), q_test.x()) == 0);
//   mu_check(fltcmp(testPose.q.y(), q_test.y()) == 0);
//   mu_check(fltcmp(testPose.q.z(), q_test.z()) == 0);
//   mu_check(fltcmp(testPose.q.w(), q_test.w()) == 0);
//
//   mu_check(fltcmp(testPose.position(0), x) == 0);
//   mu_check(fltcmp(testPose.position(1), y) == 0);
//   mu_check(fltcmp(testPose.position(2), z) == 0);
//
//   // test that rotation matrix makes sense
//   roll = M_PI / 2;
//   pitch = 0.0;
//   yaw = M_PI / 2;
//
//   testPose = Pose(roll, pitch, yaw, x, y, z);
//   rotation_mtx = testPose.rotationMatrix();
//   euler2Quaternion(roll, pitch, yaw, q_test);
//   mu_check(rotation_mtx == q_test.toRotationMatrix());
//
//   return 0;
// }

TEST(Utils, deg2radAndrad2deg) {
  double d_deg;
  double d_rad;

  d_deg = 10;
  d_rad = deg2rad(d_deg);
  ASSERT_FLOAT_EQ(d_deg, rad2deg(d_rad));
}

TEST(Utils, euler2Quaternion) {
  float roll;
  float pitch;
  float yaw;
  Eigen::Quaterniond q;

  // check identity quat is returned
  roll = 0;
  pitch = 0;
  yaw = 0;

  euler2Quaternion(roll, pitch, yaw, q);
  ASSERT_FLOAT_EQ(0.0, q.x());
  ASSERT_FLOAT_EQ(0.0, q.y());
  ASSERT_FLOAT_EQ(0.0, q.z());
  ASSERT_FLOAT_EQ(1.0, q.w());

  // check valid quat is returned
  roll = M_PI / 2;
  pitch = M_PI;
  yaw = -M_PI / 2;

  euler2Quaternion(roll, pitch, yaw, q);
  ASSERT_FLOAT_EQ(0.5, q.x());
  ASSERT_FLOAT_EQ(0.5, q.y());
  ASSERT_FLOAT_EQ(-0.5, q.z());
  ASSERT_FLOAT_EQ(-0.5, q.w());
}


TEST(Utils, euler2RotationMatrix) {
  Eigen::Matrix3d rot;
  double roll;
  double pitch;
  double yaw;

  double r01, r02, r03;
  double r11, r12, r13;
  double r21, r22, r23;

  // test roll, pitch, yaw set to 0
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;

  euler2RotationMatrix(roll, pitch, yaw, rot);

  r01 = 1;
  r02 = 0;
  r03 = 0;

  r11 = 0;
  r12 = 1;
  r13 = 0;

  r21 = 0;
  r22 = 0;
  r23 = 1;

  ASSERT_FLOAT_EQ(r01, rot(0, 0));
  ASSERT_FLOAT_EQ(r02, rot(0, 1));
  ASSERT_FLOAT_EQ(r03, rot(0, 2));

  ASSERT_FLOAT_EQ(r11, rot(1, 0));
  ASSERT_FLOAT_EQ(r12, rot(1, 1));
  ASSERT_FLOAT_EQ(r13, rot(1, 2));

  ASSERT_FLOAT_EQ(r21, rot(2, 0));
  ASSERT_FLOAT_EQ(r22, rot(2, 1));
  ASSERT_FLOAT_EQ(r23, rot(2, 2));


  // test roll
  roll = M_PI;
  pitch = 0.0;
  yaw = 0.0;

  euler2RotationMatrix(roll, pitch, yaw, rot);

  r01 = 1;
  r02 = 0;
  r03 = 0;

  r11 = 0;
  r12 = -1;
  r13 = 0;

  r21 = 0;
  r22 = 0;
  r23 = -1;

  ASSERT_FLOAT_EQ(r01, rot(0, 0));
  ASSERT_FLOAT_EQ(r02, rot(0, 1));
  ASSERT_FLOAT_EQ(r03, rot(0, 2));

  ASSERT_FLOAT_EQ(r11, rot(1, 0));
  ASSERT_FLOAT_EQ(r12, rot(1, 1));
  ASSERT_FLOAT_EQ(r13, rot(1, 2));

  ASSERT_FLOAT_EQ(r21, rot(2, 0));
  ASSERT_FLOAT_EQ(r22, rot(2, 1));
  ASSERT_FLOAT_EQ(r23, rot(2, 2));

  // test roll and pitch
  roll = M_PI;
  pitch = M_PI / 2;
  yaw = 0.0;

  euler2RotationMatrix(roll, pitch, yaw, rot);

  r01 = 0;
  r02 = 0;
  r03 = -1;

  r11 = 0;
  r12 = -1;
  r13 = 0;

  r21 = -1;
  r22 = 0;
  r23 = 0;

  ASSERT_FLOAT_EQ(r01, rot(0, 0));
  ASSERT_FLOAT_EQ(r02, rot(0, 1));
  ASSERT_FLOAT_EQ(r03, rot(0, 2));

  ASSERT_FLOAT_EQ(r11, rot(1, 0));
  ASSERT_FLOAT_EQ(r12, rot(1, 1));
  ASSERT_FLOAT_EQ(r13, rot(1, 2));

  ASSERT_FLOAT_EQ(r21, rot(2, 0));
  ASSERT_FLOAT_EQ(r22, rot(2, 1));
  ASSERT_FLOAT_EQ(r23, rot(2, 2));


  // test roll, pitch and yaw
  roll = M_PI;
  pitch = -M_PI / 2;
  yaw = M_PI / 3;

  euler2RotationMatrix(roll, pitch, yaw, rot);
  // std::cout << rot << std::endl;
  r01 = 0;
  r02 = 0.866025;
  r03 = 0.5;
  r11 = 0;
  r12 = -0.5;
  r13 = 0.866025;
  r21 = 1;
  r22 = 0;
  r23 = 0;

  ASSERT_FLOAT_EQ(r01, rot(0, 0));
  ASSERT_FLOAT_EQ(r02, rot(0, 1));
  ASSERT_FLOAT_EQ(r03, rot(0, 2));

  ASSERT_FLOAT_EQ(r11, rot(1, 0));
  ASSERT_FLOAT_EQ(r12, rot(1, 1));
  ASSERT_FLOAT_EQ(r13, rot(1, 2));

  ASSERT_FLOAT_EQ(r21, rot(2, 0));
  ASSERT_FLOAT_EQ(r22, rot(2, 1));
  ASSERT_FLOAT_EQ(r23, rot(2, 2));

  return 0;
}

TEST(Utils, linreg) {
  Eigen::Vector2d p;
  std::vector<Eigen::Vector2d> points;
  double m;
  double c;
  double r;

  p << 1, 4;
  points.push_back(p);
  p << 2, 6;
  points.push_back(p);
  p << 4, 12;
  points.push_back(p);
  p << 5, 15;
  points.push_back(p);
  p << 10, 34;
  points.push_back(p);
  p << 20, 68;
  points.push_back(p);

  linreg(points, &m, &c, &r);
  std::cout << "m: " << m << std::endl;
  std::cout << "c: " << c << std::endl;
  std::cout << "r: " << r << std::endl;
}

TEST(Utils, tic_toc(void) {
  struct timespec start;

  tic(&start);
  usleep(10 * 1000);
  printf("%f\n", toc(&start));
  printf("%f\n", mtoc(&start));
  ASSERT_TRUE(toc(&start) < 0.011);
  ASSERT_TRUE(toc(&start) > 0.009);
  ASSERT_TRUE(mtoc(&start) < 11.0);
  ASSERT_TRUE(mtoc(&start) > 9.0);
}

}  // end of awesomo namepsace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
