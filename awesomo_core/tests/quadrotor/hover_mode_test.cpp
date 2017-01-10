#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/quadrotor/hover_mode.hpp"

#define TEST_CONFIG "tests/configs/quadrotor/modes/hover_mode.yaml"


namespace awesomo {

TEST(HoverMode, constructor) {
  HoverMode mode;

  ASSERT_FALSE(mode.configured);
  ASSERT_FLOAT_EQ(0.0, mode.hover_position[0]);
  ASSERT_FLOAT_EQ(0.0, mode.hover_position[1]);
  ASSERT_FLOAT_EQ(0.0, mode.hover_height);
}

TEST(HoverMode, configure) {
  HoverMode mode;

  mode.configure(TEST_CONFIG);
  ASSERT_TRUE(mode.configured);
  ASSERT_FLOAT_EQ(1.0, mode.hover_position[0]);
  ASSERT_FLOAT_EQ(2.0, mode.hover_position[1]);
  ASSERT_FLOAT_EQ(3.0, mode.hover_height);
}

TEST(HoverMode, updateHoverXYPosition) {
  HoverMode mode;

  mode.configure(TEST_CONFIG);
  mode.updateHoverXYPosition(10.0, 20.0);
  ASSERT_FLOAT_EQ(10.0, mode.hover_position[0]);
  ASSERT_FLOAT_EQ(20.0, mode.hover_position[1]);
  ASSERT_FLOAT_EQ(3.0, mode.hover_height);
}

TEST(HoverMode, updateHoverPosition) {
  HoverMode mode;
  Vec3 position;

  mode.configure(TEST_CONFIG);
  position << 10.0, 20.0, 30.0;
  mode.updateHoverPosition(position);
  ASSERT_FLOAT_EQ(10.0, mode.hover_position[0]);
  ASSERT_FLOAT_EQ(20.0, mode.hover_position[1]);
  ASSERT_FLOAT_EQ(30.0, mode.hover_height);
}

TEST(HoverMode, getHoverPosition) {
  HoverMode mode;
  Vec3 position;

  mode.configure(TEST_CONFIG);
  position = mode.getHoverPosition();
  ASSERT_FLOAT_EQ(1.0, position(0));
  ASSERT_FLOAT_EQ(2.0, position(1));
  ASSERT_FLOAT_EQ(3.0, position(2));
}

}  // end of awesomo namespace
