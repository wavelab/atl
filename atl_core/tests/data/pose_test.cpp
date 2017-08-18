#include "atl/atl_test.hpp"
#include "atl/data/pose.hpp"

namespace atl {

TEST(Utils_data_Pose, checkPose) {
  Pose testPose;
  Quaternion q_test;
  Vec3 euler;
  Vec3 position_test;
  Mat3 rotation_mtx;
  Mat3 rotation_mtx_test;

  float roll, pitch, yaw;
  float x, y, z;

  // check Pose with no arguments (q = identity, position = 0);
  EXPECT_FLOAT_EQ(0.0, testPose.orientation.x());
  EXPECT_FLOAT_EQ(0.0, testPose.orientation.y());
  EXPECT_FLOAT_EQ(0.0, testPose.orientation.z());
  EXPECT_FLOAT_EQ(1.0, testPose.orientation.w());

  EXPECT_FLOAT_EQ(0.0, testPose.position(0));
  EXPECT_FLOAT_EQ(0.0, testPose.position(1));
  EXPECT_FLOAT_EQ(0.0, testPose.position(2));

  // test initalizeing with floats,
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  x = 2.0;
  y = 3.0;
  z = 22.0;

  testPose = Pose(roll, pitch, yaw, x, y, z);
  EXPECT_FLOAT_EQ(0, testPose.orientation.x());
  EXPECT_FLOAT_EQ(0, testPose.orientation.y());
  EXPECT_FLOAT_EQ(0, testPose.orientation.z());
  EXPECT_FLOAT_EQ(1, testPose.orientation.w());

  EXPECT_FLOAT_EQ(x, testPose.position(0));
  EXPECT_FLOAT_EQ(y, testPose.position(1));
  EXPECT_FLOAT_EQ(z, testPose.position(2));

  // test initializing non zero roll, pitch, yaw
  roll = 10.0;
  pitch = 15.0;
  yaw = -90.0;

  euler << roll, pitch, yaw;
  euler2quat(euler, 321, q_test);
  testPose = Pose(roll, pitch, yaw, x, y, z);

  EXPECT_FLOAT_EQ(q_test.x(), testPose.orientation.x());
  EXPECT_FLOAT_EQ(q_test.y(), testPose.orientation.y());
  EXPECT_FLOAT_EQ(q_test.z(), testPose.orientation.z());
  EXPECT_FLOAT_EQ(q_test.w(), testPose.orientation.w());

  EXPECT_FLOAT_EQ(x, testPose.position(0));
  EXPECT_FLOAT_EQ(y, testPose.position(1));
  EXPECT_FLOAT_EQ(z, testPose.position(2));

  // test inializing with quaterion and a postion vector
  testPose = Pose(Vec3(x, y, z), q_test);
  EXPECT_FLOAT_EQ(q_test.x(), testPose.orientation.x());
  EXPECT_FLOAT_EQ(q_test.y(), testPose.orientation.y());
  EXPECT_FLOAT_EQ(q_test.z(), testPose.orientation.z());
  EXPECT_FLOAT_EQ(q_test.w(), testPose.orientation.w());

  EXPECT_FLOAT_EQ(x, testPose.position(0));
  EXPECT_FLOAT_EQ(y, testPose.position(1));
  EXPECT_FLOAT_EQ(z, testPose.position(2));

  // test that rotation matrix makes sense
  roll = M_PI / 2;
  pitch = 0.0;
  yaw = M_PI / 2;

  testPose = Pose(roll, pitch, yaw, x, y, z);
  rotation_mtx = testPose.rotationMatrix();
  euler << roll, pitch, yaw;
  euler2quat(euler, 321, q_test);
  EXPECT_TRUE(rotation_mtx == q_test.toRotationMatrix());
}

} // namespace atl
