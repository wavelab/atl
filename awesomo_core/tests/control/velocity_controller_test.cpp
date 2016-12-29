#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/control/velocity_controller.hpp"

#define TEST_CONFIG "tests/configs/control/velocity_controller.yaml"


namespace awesomo {

TEST(VelocityController, constructor) {
  VelocityController controller;

  ASSERT_FALSE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.vx_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.vx_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.vx_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.vy_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.vy_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.vy_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.vz_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.vz_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.vz_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.hover_throttle);

  ASSERT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  ASSERT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  ASSERT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  for (int i = 0; i < 4; i++) {
    ASSERT_FLOAT_EQ(0.0, controller.setpoints[i]);
    ASSERT_FLOAT_EQ(0.0, controller.outputs[i]);
  }
}

TEST(VelocityController, configure) {
  VelocityController controller;

  controller.configure(TEST_CONFIG);

  ASSERT_TRUE(controller.configured);

  ASSERT_FLOAT_EQ(0.1, controller.vx_controller.k_p);
  ASSERT_FLOAT_EQ(0.2, controller.vx_controller.k_i);
  ASSERT_FLOAT_EQ(0.3, controller.vx_controller.k_d);

  ASSERT_FLOAT_EQ(0.1, controller.vy_controller.k_p);
  ASSERT_FLOAT_EQ(0.2, controller.vy_controller.k_i);
  ASSERT_FLOAT_EQ(0.3, controller.vy_controller.k_d);

  ASSERT_FLOAT_EQ(0.1, controller.vz_controller.k_p);
  ASSERT_FLOAT_EQ(0.2, controller.vz_controller.k_i);
  ASSERT_FLOAT_EQ(0.3, controller.vz_controller.k_d);

  ASSERT_FLOAT_EQ(0.6, controller.hover_throttle);

  ASSERT_FLOAT_EQ(deg2rad(-50.0), controller.roll_limit[0]);
  ASSERT_FLOAT_EQ(deg2rad(50.0), controller.roll_limit[1]);

  ASSERT_FLOAT_EQ(deg2rad(-50.0), controller.pitch_limit[0]);
  ASSERT_FLOAT_EQ(deg2rad(50.0), controller.pitch_limit[1]);

  for (int i = 0; i < 4; i++) {
    ASSERT_FLOAT_EQ(0.0, controller.setpoints[i]);
    ASSERT_FLOAT_EQ(0.0, controller.outputs[i]);
  }
}

TEST(VelocityController, calculate) {
  Vec3 setpoint, actual;
  float yaw_setpoint, dt;
  VelocityController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HOVERING PID OUTPUT
  setpoint << 0, 0, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;
  controller.calculate(setpoint, actual, yaw_setpoint, dt);

  // clang-format off
  // std::cout << controller.outputs[0] << "\t"
  //           << controller.outputs[1] << "\t"
  //           << controller.outputs[3] << std::endl;
  // clang-format on

  ASSERT_FLOAT_EQ(0.0, controller.outputs[0]);
  ASSERT_FLOAT_EQ(0.0, controller.outputs[1]);
  ASSERT_FLOAT_EQ(controller.hover_throttle, controller.outputs[3]);

  // CHECK MOVING TOWARDS THE X LOCATION
  setpoint << 1, 0, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint, actual, yaw_setpoint, dt);

  // clang-format off
  std::cout << controller.outputs[0] << "\t"
            << controller.outputs[1] << "\t"
            << controller.outputs[3] << std::endl;
  // clang-format on

  ASSERT_FLOAT_EQ(0.0, controller.outputs[0]);
  ASSERT_TRUE(controller.outputs[1] < 0.0);

  // CHECK MOVING TOWARDS THE Y LOCATION
  setpoint << 0, 1, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint, actual, yaw_setpoint, dt);

  // clang-format off
  std::cout << controller.outputs[0] << "\t"
            << controller.outputs[1] << "\t"
            << controller.outputs[3] << std::endl;
  // clang-format on

  ASSERT_TRUE(controller.outputs[0] > 0.0);
  ASSERT_FLOAT_EQ(0.0, controller.outputs[1]);

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  setpoint << 1, -1, 0;
  actual << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  // clang-format off
  std::cout << controller.outputs[0] << "\t"
            << controller.outputs[1] << "\t"
            << controller.outputs[3] << std::endl;
  // clang-format on

  controller.reset();
  controller.calculate(setpoint, actual, yaw_setpoint, dt);
  ASSERT_TRUE(controller.outputs[0] < 0.0);
  ASSERT_TRUE(controller.outputs[1] < 0.0);
  // ASSERT_TRUE(controller.throttle > controller.hover_throttle);
}

}  // end of awesomo namepsace
