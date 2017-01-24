#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/utils/math.hpp"


namespace awesomo {

TEST(Utils, deg2radAndrad2deg) {
  double d_deg;
  double d_rad;

  d_deg = 10;
  d_rad = deg2rad(d_deg);
  ASSERT_FLOAT_EQ(d_deg, rad2deg(d_rad));
}

TEST(Utils, euler2quat) {
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
  euler2quat(euler, 321, q);
  ASSERT_FLOAT_EQ(0.0, q.x());
  ASSERT_FLOAT_EQ(0.0, q.y());
  ASSERT_FLOAT_EQ(0.0, q.z());
  ASSERT_FLOAT_EQ(1.0, q.w());

  // check valid quat is returned
  roll = M_PI / 2;
  pitch = M_PI;
  yaw = -M_PI / 2;

  euler << roll, pitch, yaw;
  euler2quat(euler, 321, q);
  ASSERT_FLOAT_EQ(0.5, q.x());
  ASSERT_FLOAT_EQ(0.5, q.y());
  ASSERT_FLOAT_EQ(-0.5, q.z());
  ASSERT_FLOAT_EQ(-0.5, q.w());
}


TEST(Utils, euler2rot) {
  double roll;
  double pitch;
  double yaw;
  Vec3 euler;
  Mat3 rot;

  double r01, r02, r03;
  double r11, r12, r13;
  double r21, r22, r23;

  // test roll, pitch, yaw set to 0
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  euler << roll, pitch, yaw;
  euler2rot(euler, 321, rot);

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

TEST(Utils, enu2nwu) {
  Vec3 enu, nwu;

  enu << 1.0, 2.0, 3.0;
  enu2nwu(enu, nwu);

  ASSERT_FLOAT_EQ(2.0, nwu(0));
  ASSERT_FLOAT_EQ(-1.0, nwu(1));
  ASSERT_FLOAT_EQ(3.0, nwu(2));
}

TEST(Utils, nwu2enu) {
  Vec3 enu, nwu;

  nwu << 1.0, 2.0, 3.0;
  nwu2enu(nwu, enu);

  ASSERT_FLOAT_EQ(-2.0, enu(0));
  ASSERT_FLOAT_EQ(1.0, enu(1));
  ASSERT_FLOAT_EQ(3.0, enu(2));
}

TEST(Utils, cf2enu) {
  Vec3 cf, enu;

  cf << 1.0, 2.0, 3.0;
  cf2enu(cf, enu);

  ASSERT_FLOAT_EQ(1.0, enu(0));
  ASSERT_FLOAT_EQ(3.0, enu(1));
  ASSERT_FLOAT_EQ(-2.0, enu(2));
}

TEST(Utils, target2body) {
  Vec3 target_pos_if;
  Vec3 body_pos_if;
  Vec3 euler;
  Quaternion quat;
  Vec3 target_pos_bf;

  // setup
  target_pos_if << 2.0, 1.0, 0.0;
  body_pos_if << 1.0, 2.0, 0.0;

  // TEST EULER VERSION OF target2body()
  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
  std::cout << target_pos_bf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
  std::cout << target_pos_bf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
  std::cout << target_pos_bf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  target2body(target_pos_if, body_pos_if, euler, target_pos_bf);
  std::cout << target_pos_bf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // TEST QUATERNION VERSION OF target2body()
  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  euler2quat(euler, 123, quat);
  target2body(target_pos_if, body_pos_if, quat, target_pos_bf);

  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  euler2quat(euler, 123, quat);
  target2body(target_pos_if, body_pos_if, quat, target_pos_bf);

  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  euler2quat(euler, 321, quat);
  target2body(target_pos_if, body_pos_if, quat, target_pos_bf);

  ASSERT_FLOAT_EQ(1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  euler2quat(euler, 321, quat);
  target2body(target_pos_if, body_pos_if, quat, target_pos_bf);

  ASSERT_FLOAT_EQ(1.0, target_pos_bf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bf(2));
}

TEST(Utils, target2bodyplanar) {
  Vec3 target_pos_if;
  Vec3 body_pos_if;
  Vec3 euler;
  Quaternion quat;
  Vec3 target_pos_bpf;

  // setup
  target_pos_if << 2.0, 1.0, 0.0;
  body_pos_if << 1.0, 2.0, 0.0;

  // TEST EULER VERSION OF target2body()
  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  target2bodyplanar(target_pos_if, body_pos_if, euler, target_pos_bpf);
  std::cout << target_pos_bpf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  target2bodyplanar(target_pos_if, body_pos_if, euler, target_pos_bpf);
  std::cout << target_pos_bpf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  target2bodyplanar(target_pos_if, body_pos_if, euler, target_pos_bpf);
  std::cout << target_pos_bpf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  target2bodyplanar(target_pos_if, body_pos_if, euler, target_pos_bpf);
  std::cout << target_pos_bpf.transpose() << std::endl;

  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // TEST QUATERNION VERSION OF target2bodyplanar()
  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  euler2quat(euler, 123, quat);
  target2bodyplanar(target_pos_if, body_pos_if, quat, target_pos_bpf);

  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  euler2quat(euler, 123, quat);
  target2bodyplanar(target_pos_if, body_pos_if, quat, target_pos_bpf);

  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  euler2quat(euler, 321, quat);
  target2bodyplanar(target_pos_if, body_pos_if, quat, target_pos_bpf);

  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  euler2quat(euler, 321, quat);
  target2bodyplanar(target_pos_if, body_pos_if, quat, target_pos_bpf);

  ASSERT_FLOAT_EQ(1.0, target_pos_bpf(0));
  ASSERT_FLOAT_EQ(-1.0, target_pos_bpf(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_bpf(2));
}

TEST(Utils, target2inertial) {
  Vec3 target_pos_bf, target_pos_if, body_pos_if, euler;
  Quaternion quat;

  // setup
  target_pos_bf << 2.0, 1.0, 0.0;
  body_pos_if << 1.0, 2.0, 0.0;

  // TEST EULER VERSION OF target2inertial()
  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  target2inertial(target_pos_bf, body_pos_if, euler, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(0.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(4.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  target2inertial(target_pos_bf, body_pos_if, euler, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  target2inertial(target_pos_bf, body_pos_if, euler, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(2.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  target2inertial(target_pos_bf, body_pos_if, euler, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(3.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(3.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // TEST QUATERNION VERSION OF target2inertial()
  // test 0 degree
  euler << 0.0, 0.0, deg2rad(0.0);
  euler2quat(euler, 321, quat);
  target2inertial(target_pos_bf, body_pos_if, quat, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(0.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(4.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // test 90 degree
  euler << 0.0, 0.0, deg2rad(90.0);
  euler2quat(euler, 321, quat);
  target2inertial(target_pos_bf, body_pos_if, quat, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(-1.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(1.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // test 180 degree
  euler << 0.0, 0.0, deg2rad(180.0);
  euler2quat(euler, 321, quat);
  target2inertial(target_pos_bf, body_pos_if, quat, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(2.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));

  // test 270 degree
  euler << 0.0, 0.0, deg2rad(270.0);
  euler2quat(euler, 321, quat);
  target2inertial(target_pos_bf, body_pos_if, quat, target_pos_if);
  std::cout << target_pos_if.transpose() << std::endl;

  ASSERT_FLOAT_EQ(3.0, target_pos_if(0));
  ASSERT_FLOAT_EQ(3.0, target_pos_if(1));
  ASSERT_FLOAT_EQ(0.0, target_pos_if(2));
}

}  // end of awesomo namespace
