#include <gtest/gtest.h>

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

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

  r01 = 1.0;
  r02 = 0.0;
  r03 = 0.0;

  r11 = 0.0;
  r12 = 1.0;
  r13 = 0.0;

  r21 = 0.0;
  r22 = 0.0;
  r23 = 1.0;

  ASSERT_FLOAT_EQ(r01, rot(0, 0));
  ASSERT_FLOAT_EQ(r02, rot(0, 1));
  ASSERT_FLOAT_EQ(r03, rot(0, 2));

  ASSERT_FLOAT_EQ(r11, rot(1, 0));
  ASSERT_FLOAT_EQ(r12, rot(1, 1));
  ASSERT_FLOAT_EQ(r13, rot(1, 2));

  ASSERT_FLOAT_EQ(r21, rot(2, 0));
  ASSERT_FLOAT_EQ(r22, rot(2, 1));
  ASSERT_FLOAT_EQ(r23, rot(2, 2));
}

}  // end of awesomo namespace
