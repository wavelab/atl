#include <gtest/gtest.h>

#include "awesomo_core/quadrotor/quadrotor.hpp"

#define TEST_CONFIG_PATH "tests/configs/quadrotor"

TEST(Quadrotor, constructor) {
  awesomo::Quadrotor quadrotor;

  ASSERT_FALSE(quadrotor.configured);
  ASSERT_EQ(awesomo::HOVER_MODE, quadrotor.current_mode);
}

TEST(Quadrotor, configure) {
  awesomo::Quadrotor quadrotor;

  quadrotor.configure(TEST_CONFIG_PATH);
  ASSERT_TRUE(quadrotor.configured);
  ASSERT_EQ(awesomo::HOVER_MODE, quadrotor.current_mode);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
