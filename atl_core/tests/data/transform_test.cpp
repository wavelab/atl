#include "atl/atl_test.hpp"
#include "atl/data/pose.hpp"
#include "atl/data/transform.hpp"

namespace atl {

// TEST(Transform, nedToNwu) {
//   const Vec3 x_ned{1.0, 2.0, 3.0};
//   const Vec3 x_nwu = T_nwu_ned * x_ned;
//   EXPECT_TRUE(x_nwu.isApprox(Vec3{1.0, -2.0, -3.0}));
// }

TEST(Transform, inertialToBody) {
  const Vec3 x_W{1.0, 2.0, 3.0};
  Vec3 rpy;
  Vec3 x_B;

  // 0 degree
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_B = T_B_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{1.0, 2.0, 3.0}));

  // 90 degree
  rpy << 0.0, 0.0, deg2rad(90.0);
  x_B = T_B_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{2.0, -1.0, 3.0}));

  // 180 degree
  rpy << 0.0, 0.0, deg2rad(180.0);
  x_B = T_B_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{-1.0, -2.0, 3.0}));

  // -90 degree
  rpy << 0.0, 0.0, deg2rad(-90.0);
  x_B = T_B_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{-2.0, 1.0, 3.0}));

  // 0 degree + (1.0, 2.0, 3.0)
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_B = T_B_W(rpy, Vec3{1.0, 2.0, 3.0}) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{0.0, 0.0, 0.0}));
}

TEST(Transform, inertialToBodyPlanar) {
  const Vec3 x_W{1.0, 2.0, 3.0};
  Vec3 rpy;
  Vec3 x_B;

  // 0 degree
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_B = T_P_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{1.0, 2.0, 3.0}));

  // 90 degree
  rpy << 0.0, 0.0, deg2rad(90.0);
  x_B = T_P_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{2.0, -1.0, 3.0}));

  // 180 degree
  rpy << 0.0, 0.0, deg2rad(180.0);
  x_B = T_P_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{-1.0, -2.0, 3.0}));

  // -90 degree
  rpy << 0.0, 0.0, deg2rad(-90.0);
  x_B = T_P_W(rpy) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{-2.0, 1.0, 3.0}));

  // 0 degree + (1.0, 2.0, 3.0)
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_B = T_P_W(rpy, Vec3{1.0, 2.0, 3.0}) * x_W;
  EXPECT_TRUE(x_B.isApprox(Vec3{0.0, 0.0, 0.0}));
}

TEST(Transform, bodyToInertial) {
  const Vec3 x_B{1.0, 2.0, 3.0};
  Vec3 rpy;
  Vec3 x_W;

  // 0 degree
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_W = T_W_B(rpy) * x_B;
  EXPECT_TRUE(x_W.isApprox(Vec3{1.0, 2.0, 3.0}));

  // 90 degree
  rpy << 0.0, 0.0, deg2rad(90.0);
  x_W = T_W_B(rpy) * x_B;
  EXPECT_TRUE(x_W.isApprox(Vec3{-2.0, 1.0, 3.0}));

  // 180 degree
  rpy << 0.0, 0.0, deg2rad(180.0);
  x_W = T_W_B(rpy) * x_B;
  EXPECT_TRUE(x_W.isApprox(Vec3{-1.0, -2.0, 3.0}));

  // -90 degree
  rpy << 0.0, 0.0, deg2rad(-90.0);
  x_W = T_W_B(rpy) * x_B;
  EXPECT_TRUE(x_W.isApprox(Vec3{2.0, -1.0, 3.0}));

  // 0 degree + (1.0, 2.0, 3.0)
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_W = T_W_B(rpy, Vec3{1.0, 2.0, 3.0}) * x_B;
  EXPECT_TRUE(x_W.isApprox(Vec3{2.0, 4.0, 6.0}));
}

TEST(Transform, sandbox) {
  double roll = deg2rad(10.0);
  double pitch = deg2rad(20.0);
  double yaw = deg2rad(30.0);
  Vec3 euler{roll, pitch, yaw};

  // NWU
  Quaternion q_nwu;
  q_nwu = euler321ToQuat(euler);

  // NWU to NED
  Quaternion q_ned = nwu2ned(q_nwu);
  euler = quatToEuler321(q_ned);
  EXPECT_FLOAT_EQ(10, rad2deg(euler(0)));
  EXPECT_FLOAT_EQ(-20, rad2deg(euler(1)));
  EXPECT_FLOAT_EQ(-30, rad2deg(euler(2)));

  // NED to NWU
  q_nwu = ned2nwu(q_ned);
  euler = quatToEuler321(q_nwu);
  EXPECT_FLOAT_EQ(10, rad2deg(euler(0)));
  EXPECT_FLOAT_EQ(20, rad2deg(euler(1)));
  EXPECT_FLOAT_EQ(30, rad2deg(euler(2)));
}

TEST(Math, euler321ToQuat) {
  float roll;
  float pitch;
  float yaw;
  Vec3 euler;
  Quaternion q;

  // check identity quat is returned
  roll = 0;
  pitch = 0;
  yaw = 0;

  euler << roll, pitch, yaw;
  q = euler321ToQuat(euler);
  EXPECT_FLOAT_EQ(0.0, q.x());
  EXPECT_FLOAT_EQ(0.0, q.y());
  EXPECT_FLOAT_EQ(0.0, q.z());
  EXPECT_FLOAT_EQ(1.0, q.w());

  // check valid quat is returned
  roll = M_PI / 2;
  pitch = M_PI;
  yaw = -M_PI / 2;

  euler << roll, pitch, yaw;
  q = euler321ToQuat(euler);
  EXPECT_FLOAT_EQ(0.5, q.x());
  EXPECT_FLOAT_EQ(0.5, q.y());
  EXPECT_FLOAT_EQ(-0.5, q.z());
  EXPECT_FLOAT_EQ(-0.5, q.w());
}

TEST(Transform, euler321ToRot) {
  // test roll, pitch, yaw set to 0
  const double roll = 0.0;
  const double pitch = 0.0;
  const double yaw = 0.0;
  const Vec3 euler{roll, pitch, yaw};
  const Mat3 rot = euler321ToRot(euler);

  const double r01 = 1.0;
  const double r02 = 0.0;
  const double r03 = 0.0;

  const double r11 = 0.0;
  const double r12 = 1.0;
  const double r13 = 0.0;

  const double r21 = 0.0;
  const double r22 = 0.0;
  const double r23 = 1.0;

  EXPECT_FLOAT_EQ(r01, rot(0, 0));
  EXPECT_FLOAT_EQ(r02, rot(0, 1));
  EXPECT_FLOAT_EQ(r03, rot(0, 2));

  EXPECT_FLOAT_EQ(r11, rot(1, 0));
  EXPECT_FLOAT_EQ(r12, rot(1, 1));
  EXPECT_FLOAT_EQ(r13, rot(1, 2));

  EXPECT_FLOAT_EQ(r21, rot(2, 0));
  EXPECT_FLOAT_EQ(r22, rot(2, 1));
  EXPECT_FLOAT_EQ(r23, rot(2, 2));
}

TEST(Transform, enu2nwu) {
  Vec3 enu{1.0, 2.0, 3.0};
  Vec3 nwu = enu2nwu(enu);

  EXPECT_FLOAT_EQ(2.0, nwu(0));
  EXPECT_FLOAT_EQ(-1.0, nwu(1));
  EXPECT_FLOAT_EQ(3.0, nwu(2));
}

TEST(Transform, nwu2enu) {
  Vec3 nwu{1.0, 2.0, 3.0};
  Vec3 enu = nwu2enu(nwu);

  EXPECT_FLOAT_EQ(-2.0, enu(0));
  EXPECT_FLOAT_EQ(1.0, enu(1));
  EXPECT_FLOAT_EQ(3.0, enu(2));
}

TEST(Transform, edn2nwu) {
  Vec3 edn{1.0, 2.0, 3.0};
  Vec3 enu = edn2nwu(edn);

  EXPECT_FLOAT_EQ(3.0, enu(0));
  EXPECT_FLOAT_EQ(-1.0, enu(1));
  EXPECT_FLOAT_EQ(-2.0, enu(2));
}

TEST(Transform, transformPose) {
  Vec3 x{1.0, 0.0, 0.0};
  Pose pose("B", 0.0, 0.0, deg2rad(90.0), 1.0, 0.0, 0.0);

  std::cout << Transform{"A", "B", Mat3::Identity(), x} * pose << std::endl;
}

TEST(Transform, transformPosition) {
  Vec3 x{1.0, 0.0, 0.0};
  Position p("B", 1.0, 0.0, 0.0);

  std::cout << Transform{"A", "B", Mat3::Identity(), x} * p << std::endl;
}

TEST(Transform, transformVector3) {
  Vec3 x{1.0, 2.0, 3.0};
  Pose pose("W", 0.0, 0.0, deg2rad(0.0), 0.0, 0.0, 0.0);
  std::cout << T_P_W{pose.orientation} * x << std::endl;
}

} // namespace atl
