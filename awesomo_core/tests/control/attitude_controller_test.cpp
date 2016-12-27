#include <gtest/gtest.h>

#include "awesomo_core/control/attitude_controller.hpp"


TEST(AttitudeController, constructor) {
  awesomo::AttitudeController controller;

  ASSERT_FALSE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.roll_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.roll_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.roll_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.pitch_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.pitch_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.pitch_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.yaw_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.yaw_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.yaw_controller.k_d);

  for (int i = 0; i < 4; i++) {
    ASSERT_FLOAT_EQ(0.0, controller.setpoints[i]);
    ASSERT_FLOAT_EQ(0.0, controller.outputs[i]);
  }
}
