#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/control/pid_controller.hpp"


TEST(PID, constructor) {
  awesomo::PID controller;

  ASSERT_FLOAT_EQ(0.0, controller.error_prev);
  ASSERT_FLOAT_EQ(0.0, controller.error_sum);

  ASSERT_FLOAT_EQ(0.0, controller.error_p);
  ASSERT_FLOAT_EQ(0.0, controller.error_i);
  ASSERT_FLOAT_EQ(0.0, controller.error_d);

  ASSERT_FLOAT_EQ(0.0, controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.k_d);

  controller = awesomo::PID(1.0, 2.0, 3.0);

  ASSERT_FLOAT_EQ(0.0, controller.error_prev);
  ASSERT_FLOAT_EQ(0.0, controller.error_sum);

  ASSERT_FLOAT_EQ(0.0, controller.error_p);
  ASSERT_FLOAT_EQ(0.0, controller.error_i);
  ASSERT_FLOAT_EQ(0.0, controller.error_d);

  ASSERT_FLOAT_EQ(1.0, controller.k_p);
  ASSERT_FLOAT_EQ(2.0, controller.k_i);
  ASSERT_FLOAT_EQ(3.0, controller.k_d);
}

TEST(PID, calculate) {
  awesomo::PID controller;
  double output;

  // setup
  controller = awesomo::PID(1.0, 1.0, 1.0);

  // test and assert
  output = controller.calculate(10.0, 0.0, 0.1);

  ASSERT_FLOAT_EQ(1.0, controller.error_sum);
  ASSERT_FLOAT_EQ(10.0, controller.error_p);
  ASSERT_FLOAT_EQ(1.0, controller.error_i);
  ASSERT_FLOAT_EQ(100.0, controller.error_d);
  ASSERT_FLOAT_EQ(10.0, controller.error_prev);
  ASSERT_FLOAT_EQ(111.0, output);
}

TEST(PID, reset) {
  awesomo::PID controller;

  controller.error_prev = 0.1;
  controller.error_sum = 0.2;

  controller.error_p = 0.3;
  controller.error_i = 0.4;
  controller.error_d = 0.5;

  controller.reset();

  ASSERT_FLOAT_EQ(0.0, controller.error_prev);
  ASSERT_FLOAT_EQ(0.0, controller.error_sum);

  ASSERT_FLOAT_EQ(0.0, controller.error_p);
  ASSERT_FLOAT_EQ(0.0, controller.error_i);
  ASSERT_FLOAT_EQ(0.0, controller.error_d);
}
