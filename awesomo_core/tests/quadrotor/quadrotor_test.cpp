#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/quadrotor/quadrotor.hpp"

#define TEST_CONFIG_PATH "tests/configs/quadrotor"

namespace awesomo {

TEST(Quadrotor, constructor) {
  Quadrotor quadrotor;

  ASSERT_FALSE(quadrotor.configured);
  ASSERT_EQ(DISCOVER_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, configure) {
  Quadrotor quadrotor;

  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_TRUE(quadrotor.configured);
  ASSERT_EQ(DISCOVER_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, setMode) {
  Quadrotor quadrotor;

  // check fail
  ASSERT_EQ(-1, quadrotor.setMode(HOVER_MODE));

  // make sure quadrotor is configured
  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_TRUE(quadrotor.configured);

  // check NOT_SET
  quadrotor.setMode(NOT_SET);
  ASSERT_EQ(NOT_SET, quadrotor.current_mode);

  // check DISARM_MODE
  quadrotor.setMode(DISARM_MODE);
  ASSERT_EQ(DISARM_MODE, quadrotor.current_mode);

  // check HOVER_MODE
  quadrotor.setMode(HOVER_MODE);
  ASSERT_EQ(HOVER_MODE, quadrotor.current_mode);

  // check DISCOVER_MODE
  quadrotor.setMode(DISCOVER_MODE);
  ASSERT_EQ(DISCOVER_MODE, quadrotor.current_mode);

  // check TRACKING_MODE
  quadrotor.setMode(TRACKING_MODE);
  ASSERT_EQ(TRACKING_MODE, quadrotor.current_mode);

  // check LANDING_MODE
  quadrotor.setMode(LANDING_MODE);
  ASSERT_EQ(LANDING_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, setPose) {
  Quadrotor quadrotor;
  Pose pose;

  // setup
  pose.position << 1.0, 2.0, 3.0;
  pose.orientation = Quaternion();

  // check fail
  ASSERT_EQ(-1, quadrotor.setPose(pose));

  // check set pose
  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, quadrotor.setPose(pose));
  ASSERT_FLOAT_EQ(1.0, quadrotor.pose.position(0));
  ASSERT_FLOAT_EQ(2.0, quadrotor.pose.position(1));
  ASSERT_FLOAT_EQ(3.0, quadrotor.pose.position(2));
}

TEST(Quadrotor, setVelocity) {
  Quadrotor quadrotor;
  Vec3 velocity;

  // setup
  velocity << 1.0, 2.0, 3.0;

  // check fail
  ASSERT_EQ(-1, quadrotor.setVelocity(velocity));

  // check set velocity
  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, quadrotor.setVelocity(velocity));
  ASSERT_FLOAT_EQ(1.0, quadrotor.velocity(0));
  ASSERT_FLOAT_EQ(2.0, quadrotor.velocity(1));
  ASSERT_FLOAT_EQ(3.0, quadrotor.velocity(2));
}

TEST(Quadrotor, setTargetPosition) {
  Quadrotor quadrotor;
  Vec3 target_position;

  // setup
  target_position << 1.0, 2.0, 3.0;

  // check fail
  ASSERT_EQ(-1, quadrotor.setTargetPosition(target_position));

  // check set target_position
  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, quadrotor.setTargetPosition(target_position));
  ASSERT_FLOAT_EQ(1.0, quadrotor.landing_target.position_bf(0));
  ASSERT_FLOAT_EQ(2.0, quadrotor.landing_target.position_bf(1));
  ASSERT_FLOAT_EQ(3.0, quadrotor.landing_target.position_bf(2));
}

TEST(Quadrotor, setTargetVelocity) {
  Quadrotor quadrotor;
  Vec3 target_velocity;

  // setup
  target_velocity << 1.0, 2.0, 3.0;

  // check fail
  ASSERT_EQ(-1, quadrotor.setTargetVelocity(target_velocity));

  // check set target_velocity
  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, quadrotor.setTargetVelocity(target_velocity));
  ASSERT_FLOAT_EQ(1.0, quadrotor.landing_target.velocity_bf(0));
  ASSERT_FLOAT_EQ(2.0, quadrotor.landing_target.velocity_bf(1));
  ASSERT_FLOAT_EQ(3.0, quadrotor.landing_target.velocity_bf(2));
}

TEST(Quadrotor, setHoverXYPosition) {
  Quadrotor quadrotor;
  Vec3 hover_position;

  // setup
  hover_position << 1.0, 2.0, 3.0;

  // check fail
  ASSERT_EQ(-1, quadrotor.setHoverXYPosition(hover_position));

  // check set hover_position
  quadrotor.configure(TEST_CONFIG_PATH);
  quadrotor.hover_position(2) = 10.0;

  ASSERT_EQ(0, quadrotor.setHoverXYPosition(hover_position));
  ASSERT_FLOAT_EQ(1.0, quadrotor.hover_position(0));
  ASSERT_FLOAT_EQ(2.0, quadrotor.hover_position(1));
  ASSERT_FLOAT_EQ(10.0, quadrotor.hover_position(2));
}

TEST(Quadrotor, setHoverPosition) {
  Quadrotor quadrotor;
  Vec3 hover_position;

  // setup
  hover_position << 11.0, 22.0, 33.0;

  // check fail
  ASSERT_EQ(-1, quadrotor.setHoverPosition(hover_position));

  // check set hover_position
  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, quadrotor.setHoverPosition(hover_position));
  ASSERT_FLOAT_EQ(11.0, quadrotor.hover_position(0));
  ASSERT_FLOAT_EQ(22.0, quadrotor.hover_position(1));
  ASSERT_FLOAT_EQ(33.0, quadrotor.hover_position(2));
}

TEST(Quadrotor, conditionsMet) {
  Quadrotor quadrotor;
  bool conditions[3];

  // setup
  conditions[0] = true;
  conditions[1] = true;
  conditions[2] = false;

  // check fail
  ASSERT_FALSE(quadrotor.conditionsMet(conditions, 3));

  // check pass
  conditions[2] = true;
  ASSERT_TRUE(quadrotor.conditionsMet(conditions, 3));
}

}  // end of awesomo namespace
