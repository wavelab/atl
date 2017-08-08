#include "atl/control/pid.hpp"
#include "atl/atl_test.hpp"

TEST(PID, constructor) {
  atl::PID controller;

  EXPECT_FLOAT_EQ(0.0, controller.error_prev);
  EXPECT_FLOAT_EQ(0.0, controller.error_sum);

  EXPECT_FLOAT_EQ(0.0, controller.error_p);
  EXPECT_FLOAT_EQ(0.0, controller.error_i);
  EXPECT_FLOAT_EQ(0.0, controller.error_d);

  EXPECT_FLOAT_EQ(0.0, controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.k_d);

  controller = atl::PID(1.0, 2.0, 3.0);

  EXPECT_FLOAT_EQ(0.0, controller.error_prev);
  EXPECT_FLOAT_EQ(0.0, controller.error_sum);

  EXPECT_FLOAT_EQ(0.0, controller.error_p);
  EXPECT_FLOAT_EQ(0.0, controller.error_i);
  EXPECT_FLOAT_EQ(0.0, controller.error_d);

  EXPECT_FLOAT_EQ(1.0, controller.k_p);
  EXPECT_FLOAT_EQ(2.0, controller.k_i);
  EXPECT_FLOAT_EQ(3.0, controller.k_d);
}

TEST(PID, update) {
  atl::PID controller{1.0, 1.0, 1.0};

  // test and assert
  double output = controller.update(10.0, 0.0, 0.1);

  EXPECT_FLOAT_EQ(1.0, controller.error_sum);
  EXPECT_FLOAT_EQ(10.0, controller.error_p);
  EXPECT_FLOAT_EQ(1.0, controller.error_i);
  EXPECT_FLOAT_EQ(100.0, controller.error_d);
  EXPECT_FLOAT_EQ(10.0, controller.error_prev);
  EXPECT_FLOAT_EQ(111.0, output);
}

TEST(PID, reset) {
  atl::PID controller;

  controller.error_prev = 0.1;
  controller.error_sum = 0.2;

  controller.error_p = 0.3;
  controller.error_i = 0.4;
  controller.error_d = 0.5;

  controller.reset();

  EXPECT_FLOAT_EQ(0.0, controller.error_prev);
  EXPECT_FLOAT_EQ(0.0, controller.error_sum);

  EXPECT_FLOAT_EQ(0.0, controller.error_p);
  EXPECT_FLOAT_EQ(0.0, controller.error_i);
  EXPECT_FLOAT_EQ(0.0, controller.error_d);
}
