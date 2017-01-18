#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/quadrotor/landing_target.hpp"

namespace awesomo {

TEST(LandingTarget, constructor) {
  LandingTarget landing_target;

  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(2));
  ASSERT_FALSE(landing_target.detected);
  ASSERT_TRUE(landing_target.losted);
  ASSERT_FLOAT_EQ(0.0, landing_target.last_seen.tv_sec);
  ASSERT_FLOAT_EQ(0.0, landing_target.last_seen.tv_nsec);

  ASSERT_FLOAT_EQ(1000.0, landing_target.lost_threshold);
}

TEST(LandingTarget, isTargetLosted) {
  LandingTarget landing_target;

  // check initial LandingTarget::losted
  ASSERT_TRUE(landing_target.losted);

  // test LandingTarget::isTargetLosted() return false
  tic(&landing_target.last_seen);
  ASSERT_FALSE(landing_target.isTargetLosted());

  // test LandingTarget::isTargetLosted() return true
  sleep(landing_target.lost_threshold / 1000.0);
  ASSERT_TRUE(landing_target.isTargetLosted());

  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(2));
  ASSERT_FALSE(landing_target.detected);
  ASSERT_TRUE(landing_target.losted);
  ASSERT_FLOAT_EQ(0.0, landing_target.first_seen.tv_sec);
  ASSERT_FLOAT_EQ(0.0, landing_target.first_seen.tv_nsec);
  ASSERT_FLOAT_EQ(0.0, landing_target.last_seen.tv_sec);
  ASSERT_FLOAT_EQ(0.0, landing_target.last_seen.tv_nsec);
}

TEST(LandingTarget, setTarget) {
  LandingTarget landing_target;
  Vec3 position, velocity;

  position << 1.0, 2.0, 3.0;
  velocity << 1.0, 2.0, 3.0;
  landing_target.setTarget(position, velocity, true);

  ASSERT_FLOAT_EQ(1.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(2.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(3.0, landing_target.position_bf(2));
  ASSERT_FLOAT_EQ(1.0, landing_target.velocity_bf(0));
  ASSERT_FLOAT_EQ(2.0, landing_target.velocity_bf(1));
  ASSERT_FLOAT_EQ(3.0, landing_target.velocity_bf(2));
  ASSERT_TRUE(landing_target.detected);
  ASSERT_FALSE(landing_target.losted);
  ASSERT_NE(0, landing_target.last_seen.tv_sec);
  ASSERT_NE(0, landing_target.last_seen.tv_nsec);
}

TEST(LandingTarget, tracked) {
  LandingTarget landing_target;
  Vec3 position, velocity;

  position << 1.0, 2.0, 3.0;
  velocity << 1.0, 2.0, 3.0;
  landing_target.setTarget(position, velocity, true);

  sleep(1);
  ASSERT_NEAR(1000.0, landing_target.tracked(), 10.0);

  sleep(1);
  ASSERT_NEAR(2000.0, landing_target.tracked(), 10.0);
}

TEST(LandingTarget, reset) {
  LandingTarget landing_target;

  landing_target.reset();

  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(2));
  ASSERT_FALSE(landing_target.detected);
  ASSERT_TRUE(landing_target.losted);
  ASSERT_FLOAT_EQ(0.0, landing_target.first_seen.tv_sec);
  ASSERT_FLOAT_EQ(0.0, landing_target.first_seen.tv_nsec);
  ASSERT_FLOAT_EQ(0.0, landing_target.last_seen.tv_sec);
  ASSERT_FLOAT_EQ(0.0, landing_target.last_seen.tv_nsec);
}

TEST(LandingTarget, update) {
  LandingTarget landing_target;
  Vec3 position, velocity;

  // 1st update
  landing_target.lost_threshold = 2000;
  landing_target.update();

  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(2));
  ASSERT_FALSE(landing_target.detected);
  ASSERT_TRUE(landing_target.losted);
  ASSERT_EQ(0, landing_target.first_seen.tv_sec);
  ASSERT_EQ(0, landing_target.first_seen.tv_nsec);
  ASSERT_EQ(0, landing_target.last_seen.tv_sec);
  ASSERT_EQ(0, landing_target.last_seen.tv_nsec);

  // 2rd update - set target position
  position << 1.0, 2.0, 3.0;
  velocity << 1.0, 2.0, 3.0;
  landing_target.setTarget(position, velocity, true);
  landing_target.update();

  ASSERT_FLOAT_EQ(1.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(2.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(3.0, landing_target.position_bf(2));
  ASSERT_TRUE(landing_target.detected);
  ASSERT_FALSE(landing_target.losted);
  ASSERT_NE(0, landing_target.first_seen.tv_sec);
  ASSERT_NE(0, landing_target.first_seen.tv_nsec);
  ASSERT_NE(0, landing_target.last_seen.tv_sec);
  ASSERT_NE(0, landing_target.last_seen.tv_nsec);

  // 3rd update - target not detected
  sleep(1);
  position << 3.0, 2.0, 1.0;
  velocity << 1.0, 2.0, 3.0;
  landing_target.setTarget(position, velocity, false);
  landing_target.update();

  ASSERT_FLOAT_EQ(3.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(2.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(1.0, landing_target.position_bf(2));
  ASSERT_FALSE(landing_target.detected);
  ASSERT_FALSE(landing_target.losted);
  ASSERT_NE(0, landing_target.first_seen.tv_sec);
  ASSERT_NE(0, landing_target.first_seen.tv_nsec);
  ASSERT_NE(0, landing_target.last_seen.tv_sec);
  ASSERT_NE(0, landing_target.last_seen.tv_nsec);

  // 4th update - target losted
  sleep(landing_target.lost_threshold / 1000.0);
  landing_target.update();

  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(0.0, landing_target.position_bf(2));
  ASSERT_FALSE(landing_target.detected);
  ASSERT_TRUE(landing_target.losted);
  ASSERT_EQ(0, landing_target.first_seen.tv_sec);
  ASSERT_EQ(0, landing_target.first_seen.tv_nsec);
  ASSERT_EQ(0, landing_target.last_seen.tv_sec);
  ASSERT_EQ(0, landing_target.last_seen.tv_nsec);
}

}  // end of awesomo namespace
