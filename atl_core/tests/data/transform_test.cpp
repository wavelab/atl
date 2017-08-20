#include "atl/atl_test.hpp"
#include "atl/data/transform.hpp"

namespace atl {

TEST(Transform, nedToNwu) {
  const Vec3 x_ned{1.0, 2.0, 3.0};
  const Vec3 x_nwu = T_nwu_ned * x_ned;
  EXPECT_TRUE(x_nwu.isApprox(Vec3{1.0, -2.0, -3.0}));
}

TEST(Transform, inertialToBody) {
  const Vec3 x_if{1.0, 2.0, 3.0};
  Vec3 rpy;
  Vec3 x_bf;

  // 0 degree
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_bf = T_bf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{1.0, 2.0, 3.0}));

  // 90 degree
  rpy << 0.0, 0.0, deg2rad(90.0);
  x_bf = T_bf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{2.0, -1.0, 3.0}));

  // 180 degree
  rpy << 0.0, 0.0, deg2rad(180.0);
  x_bf = T_bf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{-1.0, -2.0, 3.0}));

  // -90 degree
  rpy << 0.0, 0.0, deg2rad(-90.0);
  x_bf = T_bf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{-2.0, 1.0, 3.0}));

  // 0 degree + (1.0, 2.0, 3.0)
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_bf = T_bf_if(rpy, Vec3{1.0, 2.0, 3.0}) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{0.0, 0.0, 0.0}));
}

TEST(Transform, inertialToBodyPlanar) {
  const Vec3 x_if{1.0, 2.0, 3.0};
  Vec3 rpy;
  Vec3 x_bf;

  // 0 degree
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_bf = T_bpf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{1.0, 2.0, 3.0}));

  // 90 degree
  rpy << 0.0, 0.0, deg2rad(90.0);
  x_bf = T_bpf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{2.0, -1.0, 3.0}));

  // 180 degree
  rpy << 0.0, 0.0, deg2rad(180.0);
  x_bf = T_bpf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{-1.0, -2.0, 3.0}));

  // -90 degree
  rpy << 0.0, 0.0, deg2rad(-90.0);
  x_bf = T_bpf_if(rpy) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{-2.0, 1.0, 3.0}));

  // 0 degree + (1.0, 2.0, 3.0)
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_bf = T_bpf_if(rpy, Vec3{1.0, 2.0, 3.0}) * x_if;
  EXPECT_TRUE(x_bf.isApprox(Vec3{0.0, 0.0, 0.0}));
}

TEST(Transform, bodyToInertial) {
  const Vec3 x_bf{1.0, 2.0, 3.0};
  Vec3 rpy;
  Vec3 x_if;

  // 0 degree
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_if = T_if_bf(rpy) * x_bf;
  EXPECT_TRUE(x_if.isApprox(Vec3{1.0, 2.0, 3.0}));

  // 90 degree
  rpy << 0.0, 0.0, deg2rad(90.0);
  x_if = T_if_bf(rpy) * x_bf;
  EXPECT_TRUE(x_if.isApprox(Vec3{-2.0, 1.0, 3.0}));

  // 180 degree
  rpy << 0.0, 0.0, deg2rad(180.0);
  x_if = T_if_bf(rpy) * x_bf;
  EXPECT_TRUE(x_if.isApprox(Vec3{-1.0, -2.0, 3.0}));

  // -90 degree
  rpy << 0.0, 0.0, deg2rad(-90.0);
  x_if = T_if_bf(rpy) * x_bf;
  EXPECT_TRUE(x_if.isApprox(Vec3{2.0, -1.0, 3.0}));

  // 0 degree + (1.0, 2.0, 3.0)
  rpy << 0.0, 0.0, deg2rad(0.0);
  x_if = T_if_bf(rpy, Vec3{1.0, 2.0, 3.0}) * x_bf;
  EXPECT_TRUE(x_if.isApprox(Vec3{2.0, 4.0, 6.0}));
}

} // namespace atl
