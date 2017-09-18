#include "atl/data/pose.hpp"
#include "atl/atl_test.hpp"

namespace atl {

TEST(Utils_data_Pose, checkPose) {
  Pose test_pose;
  Quaternion q_test;
  Vec3 euler;
  Vec3 position_test;
  Mat3 rotation_mtx;
  Mat3 rotation_mtx_test;

  float roll, pitch, yaw;
  float x, y, z;

  // check Pose with no arguments (q = identity, position = 0);
  EXPECT_FLOAT_EQ(0.0, test_pose.orientation.x());
  EXPECT_FLOAT_EQ(0.0, test_pose.orientation.y());
  EXPECT_FLOAT_EQ(0.0, test_pose.orientation.z());
  EXPECT_FLOAT_EQ(1.0, test_pose.orientation.w());

  EXPECT_FLOAT_EQ(0.0, test_pose.position(0));
  EXPECT_FLOAT_EQ(0.0, test_pose.position(1));
  EXPECT_FLOAT_EQ(0.0, test_pose.position(2));

  // test initalizeing with floats,
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  x = 2.0;
  y = 3.0;
  z = 22.0;

  test_pose = Pose(WORLD_FRAME, roll, pitch, yaw, x, y, z);
  EXPECT_FLOAT_EQ(0, test_pose.orientation.x());
  EXPECT_FLOAT_EQ(0, test_pose.orientation.y());
  EXPECT_FLOAT_EQ(0, test_pose.orientation.z());
  EXPECT_FLOAT_EQ(1, test_pose.orientation.w());

  EXPECT_FLOAT_EQ(x, test_pose.position(0));
  EXPECT_FLOAT_EQ(y, test_pose.position(1));
  EXPECT_FLOAT_EQ(z, test_pose.position(2));

  // test initializing non zero roll, pitch, yaw
  roll = 10.0;
  pitch = 15.0;
  yaw = -90.0;

  euler << roll, pitch, yaw;
  q_test = euler321ToQuat(euler);
  test_pose = Pose(WORLD_FRAME, roll, pitch, yaw, x, y, z);

  EXPECT_FLOAT_EQ(q_test.x(), test_pose.orientation.x());
  EXPECT_FLOAT_EQ(q_test.y(), test_pose.orientation.y());
  EXPECT_FLOAT_EQ(q_test.z(), test_pose.orientation.z());
  EXPECT_FLOAT_EQ(q_test.w(), test_pose.orientation.w());

  EXPECT_FLOAT_EQ(x, test_pose.position(0));
  EXPECT_FLOAT_EQ(y, test_pose.position(1));
  EXPECT_FLOAT_EQ(z, test_pose.position(2));

  // test inializing with quaterion and a postion vector
  test_pose = Pose(WORLD_FRAME, Vec3(x, y, z), q_test);
  EXPECT_FLOAT_EQ(q_test.x(), test_pose.orientation.x());
  EXPECT_FLOAT_EQ(q_test.y(), test_pose.orientation.y());
  EXPECT_FLOAT_EQ(q_test.z(), test_pose.orientation.z());
  EXPECT_FLOAT_EQ(q_test.w(), test_pose.orientation.w());

  EXPECT_FLOAT_EQ(x, test_pose.position(0));
  EXPECT_FLOAT_EQ(y, test_pose.position(1));
  EXPECT_FLOAT_EQ(z, test_pose.position(2));

  // test that rotation matrix makes sense
  roll = M_PI / 2;
  pitch = 0.0;
  yaw = M_PI / 2;

  test_pose = Pose(WORLD_FRAME, roll, pitch, yaw, x, y, z);
  rotation_mtx = test_pose.rotationMatrix();
  euler << roll, pitch, yaw;
  q_test = euler321ToQuat(euler);
  EXPECT_TRUE(rotation_mtx == q_test.toRotationMatrix());
}

} // namespace atl
