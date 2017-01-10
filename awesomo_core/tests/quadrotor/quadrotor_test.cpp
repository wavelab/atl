#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/quadrotor/quadrotor.hpp"

#define TEST_CONFIG_PATH "tests/configs/quadrotor"

TEST(Quadrotor, constructor) {
  awesomo::Quadrotor quadrotor;

  ASSERT_FALSE(quadrotor.configured);
  ASSERT_EQ(awesomo::DISCOVER_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, configure) {
  awesomo::Quadrotor quadrotor;

  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_TRUE(quadrotor.configured);
  ASSERT_EQ(awesomo::DISCOVER_MODE, quadrotor.current_mode);
}
