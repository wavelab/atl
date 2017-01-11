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

}  // end of awesomo namespace
