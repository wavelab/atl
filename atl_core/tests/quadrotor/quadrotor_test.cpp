#include "atl/quadrotor/quadrotor.hpp"
#include "atl/atl_test.hpp"

#define TEST_CONFIG_PATH "tests/configs/quadrotor"

namespace atl {

TEST(Quadrotor, constructor) {
  Quadrotor quadrotor;

  EXPECT_FALSE(quadrotor.configured);
  EXPECT_EQ(DISCOVER_MODE, quadrotor.current_mode);

  EXPECT_FLOAT_EQ(0.0, quadrotor.recover_height);
  EXPECT_FALSE(quadrotor.auto_track);
  EXPECT_FALSE(quadrotor.auto_land);
  EXPECT_FALSE(quadrotor.auto_disarm);
  EXPECT_FLOAT_EQ(0.0, quadrotor.target_lost_threshold);
  EXPECT_FLOAT_EQ(FLT_MAX, quadrotor.min_discover_time);
  EXPECT_FLOAT_EQ(FLT_MAX, quadrotor.min_tracking_time);
}

TEST(Quadrotor, configure) {
  Quadrotor quadrotor;

  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_TRUE(quadrotor.configured);
  EXPECT_EQ(DISCOVER_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, setMode) {
  Quadrotor quadrotor;

  // check fail
  EXPECT_EQ(-1, quadrotor.setMode(HOVER_MODE));

  // make sure quadrotor is configured
  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_TRUE(quadrotor.configured);

  // check NOT_SET
  quadrotor.setMode(NOT_SET);
  EXPECT_EQ(NOT_SET, quadrotor.current_mode);

  // check DISARM_MODE
  quadrotor.setMode(DISARM_MODE);
  EXPECT_EQ(DISARM_MODE, quadrotor.current_mode);

  // check HOVER_MODE
  quadrotor.setMode(HOVER_MODE);
  EXPECT_EQ(HOVER_MODE, quadrotor.current_mode);

  // check DISCOVER_MODE
  quadrotor.setMode(DISCOVER_MODE);
  EXPECT_EQ(DISCOVER_MODE, quadrotor.current_mode);

  // check TRACKING_MODE
  quadrotor.setMode(TRACKING_MODE);
  EXPECT_EQ(TRACKING_MODE, quadrotor.current_mode);

  // check LANDING_MODE
  quadrotor.setMode(LANDING_MODE);
  EXPECT_EQ(LANDING_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, setPose) {
  Quadrotor quadrotor;
  Pose pose;

  // setup
  pose.position << 1.0, 2.0, 3.0;
  pose.orientation = Quaternion();

  // check fail
  EXPECT_EQ(-1, quadrotor.setPose(pose));

  // check set pose
  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, quadrotor.setPose(pose));
  EXPECT_FLOAT_EQ(1.0, quadrotor.pose.position(0));
  EXPECT_FLOAT_EQ(2.0, quadrotor.pose.position(1));
  EXPECT_FLOAT_EQ(3.0, quadrotor.pose.position(2));
}

TEST(Quadrotor, setVelocity) {
  Quadrotor quadrotor;
  Vec3 velocity;

  // setup
  velocity << 1.0, 2.0, 3.0;

  // check fail
  EXPECT_EQ(-1, quadrotor.setVelocity(velocity));

  // check set velocity
  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, quadrotor.setVelocity(velocity));
  EXPECT_FLOAT_EQ(1.0, quadrotor.velocity(0));
  EXPECT_FLOAT_EQ(2.0, quadrotor.velocity(1));
  EXPECT_FLOAT_EQ(3.0, quadrotor.velocity(2));
}

TEST(Quadrotor, setYaw) {
  Quadrotor quadrotor;

  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_FLOAT_EQ(0, quadrotor.yaw);

  quadrotor.setYaw(1);
  EXPECT_FLOAT_EQ(1, quadrotor.yaw);
}

TEST(Quadrotor, setTargetPosition) {
  Quadrotor quadrotor;
  Vec3 target_position;

  // setup
  target_position << 1.0, 2.0, 3.0;

  // check fail
  EXPECT_EQ(-1, quadrotor.setTargetPosition(target_position));

  // check set target_position
  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, quadrotor.setTargetPosition(target_position));
  EXPECT_FLOAT_EQ(1.0, quadrotor.landing_target.position_bf(0));
  EXPECT_FLOAT_EQ(2.0, quadrotor.landing_target.position_bf(1));
  EXPECT_FLOAT_EQ(3.0, quadrotor.landing_target.position_bf(2));
}

TEST(Quadrotor, setTargetVelocity) {
  Quadrotor quadrotor;
  Vec3 target_velocity;

  // setup
  target_velocity << 1.0, 2.0, 3.0;

  // check fail
  EXPECT_EQ(-1, quadrotor.setTargetVelocity(target_velocity));

  // check set target_velocity
  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, quadrotor.setTargetVelocity(target_velocity));
  EXPECT_FLOAT_EQ(1.0, quadrotor.landing_target.velocity_bf(0));
  EXPECT_FLOAT_EQ(2.0, quadrotor.landing_target.velocity_bf(1));
  EXPECT_FLOAT_EQ(3.0, quadrotor.landing_target.velocity_bf(2));
}

TEST(Quadrotor, setHoverXYPosition) {
  Quadrotor quadrotor;
  Vec3 hover_position;

  // setup
  hover_position << 1.0, 2.0, 3.0;

  // check fail
  EXPECT_EQ(-1, quadrotor.setHoverXYPosition(hover_position));

  // check set hover_position
  quadrotor.configure(TEST_CONFIG_PATH);
  quadrotor.hover_position(2) = 10.0;

  EXPECT_EQ(0, quadrotor.setHoverXYPosition(hover_position));
  EXPECT_FLOAT_EQ(1.0, quadrotor.hover_position(0));
  EXPECT_FLOAT_EQ(2.0, quadrotor.hover_position(1));
  EXPECT_FLOAT_EQ(10.0, quadrotor.hover_position(2));
}

TEST(Quadrotor, setHoverPosition) {
  Quadrotor quadrotor;
  Vec3 hover_position;

  // setup
  hover_position << 11.0, 22.0, 33.0;

  // check fail
  EXPECT_EQ(-1, quadrotor.setHoverPosition(hover_position));

  // check set hover_position
  quadrotor.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, quadrotor.setHoverPosition(hover_position));
  EXPECT_FLOAT_EQ(11.0, quadrotor.hover_position(0));
  EXPECT_FLOAT_EQ(22.0, quadrotor.hover_position(1));
  EXPECT_FLOAT_EQ(33.0, quadrotor.hover_position(2));
}

TEST(Quadrotor, conditionsMet) {
  Quadrotor quadrotor;
  bool conditions[3];

  // setup
  conditions[0] = true;
  conditions[1] = true;
  conditions[2] = false;

  // check fail
  EXPECT_FALSE(quadrotor.conditionsMet(conditions, 3));

  // check pass
  conditions[2] = true;
  EXPECT_TRUE(quadrotor.conditionsMet(conditions, 3));
}

}  // namespace atl
