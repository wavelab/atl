#include <gtest/gtest.h>

#include "awesomo_core/control/position_controller.hpp"

#define TEST_CONFIG "tests/configs/control/position_controller.yaml"


namespace awesomo {

TEST(PositionController, constructor) {
  PositionController controller;

  ASSERT_FALSE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.x_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.x_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.x_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.y_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.y_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.y_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.z_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.z_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.z_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.hover_throttle);

  ASSERT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  ASSERT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  ASSERT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.setpoint_x);
  ASSERT_FLOAT_EQ(0.0, controller.setpoint_y);
  ASSERT_FLOAT_EQ(0.0, controller.setpoint_z);

  ASSERT_FLOAT_EQ(0.0, controller.output_roll);
  ASSERT_FLOAT_EQ(0.0, controller.output_pitch);
  ASSERT_FLOAT_EQ(0.0, controller.output_throttle);
}

TEST(PositionController, configure) {
  PositionController controller;

  controller.configure(TEST_CONFIG);

  ASSERT_TRUE(controller.configured);

  ASSERT_FLOAT_EQ(0.1, controller.x_controller.k_p);
  ASSERT_FLOAT_EQ(0.2, controller.x_controller.k_i);
  ASSERT_FLOAT_EQ(0.3, controller.x_controller.k_d);

  ASSERT_FLOAT_EQ(0.1, controller.y_controller.k_p);
  ASSERT_FLOAT_EQ(0.2, controller.y_controller.k_i);
  ASSERT_FLOAT_EQ(0.3, controller.y_controller.k_d);

  ASSERT_FLOAT_EQ(0.1, controller.z_controller.k_p);
  ASSERT_FLOAT_EQ(0.2, controller.z_controller.k_i);
  ASSERT_FLOAT_EQ(0.3, controller.z_controller.k_d);

  ASSERT_FLOAT_EQ(0.6, controller.hover_throttle);

  ASSERT_FLOAT_EQ(deg2rad(-50.0), controller.roll_limit[0]);
  ASSERT_FLOAT_EQ(deg2rad(50.0), controller.roll_limit[1]);

  ASSERT_FLOAT_EQ(deg2rad(-50.0), controller.pitch_limit[0]);
  ASSERT_FLOAT_EQ(deg2rad(50.0), controller.pitch_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.setpoint_x);
  ASSERT_FLOAT_EQ(0.0, controller.setpoint_y);
  ASSERT_FLOAT_EQ(0.0, controller.setpoint_z);

  ASSERT_FLOAT_EQ(0.0, controller.output_roll);
  ASSERT_FLOAT_EQ(0.0, controller.output_pitch);
  ASSERT_FLOAT_EQ(0.0, controller.output_throttle);
}

TEST(PositionController, calculate) {
  Vec3 setpoint, actual;
  float yaw_setpoint, dt;
  PositionController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HOVERING PID OUTPUT
  setpoint << 0, 0, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;
  controller.calculate(setpoint, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.output_roll);
  ASSERT_FLOAT_EQ(0.0, controller.output_pitch);
  ASSERT_FLOAT_EQ(controller.hover_throttle, controller.output_throttle);

  // CHECK MOVING TOWARDS THE Y LOCATION
  setpoint << 0, 1, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.output_roll < 0.0);
  ASSERT_FLOAT_EQ(0.0, controller.output_pitch);

  // CHECK MOVING TOWARDS THE X LOCATION
  setpoint << 1, 0, 0;
  actual << 0.0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.01;

  controller.reset();
  controller.calculate(setpoint, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.output_roll);
  ASSERT_TRUE(controller.output_pitch > 0.0);

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  setpoint << 1, 1, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.output_roll < 0.0);
  ASSERT_TRUE(controller.output_pitch > 0.0);

  // CHECK MOVING YAW
  setpoint << 0, 0, 0;
  actual << 0, 0, 0;
  yaw_setpoint = deg2rad(90.0);
  dt = 0.1;

  controller.reset();
  Vec4 outputs = controller.calculate(setpoint, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(yaw_setpoint, outputs(2));
}

}  // end of awesomo namepsace
