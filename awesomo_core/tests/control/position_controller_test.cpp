#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/control/position_controller.hpp"

#define TEST_CONFIG "tests/configs/control/position_controller.yaml"


namespace awesomo {

TEST(PositionController, constructor) {
  PositionController controller;

  ASSERT_FALSE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.dt);

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

  ASSERT_FLOAT_EQ(0.0, controller.setpoints(0));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(1));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(2));

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(2));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(PositionController, configure) {
  PositionController controller;

  controller.configure(TEST_CONFIG);

  ASSERT_TRUE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.dt);

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

  ASSERT_FLOAT_EQ(0.0, controller.setpoints(0));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(1));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(2));

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(2));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(PositionController, calculate) {
  Vec3 setpoint_nwu, setpoint_enu;
  Pose actual;
  float yaw_setpoint, dt;
  PositionController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HOVERING PID OUTPUT
  setpoint_nwu << 0, 0, 0;
  nwu2enu(setpoint_nwu, setpoint_enu);
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;
  controller.calculate(setpoint_enu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(controller.hover_throttle, controller.outputs(3));

  // CHECK MOVING TOWARDS THE Y LOCATION
  setpoint_nwu << 0, 1, 0;
  nwu2enu(setpoint_nwu, setpoint_enu);
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint_enu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) < 0.0);
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));

  // CHECK MOVING TOWARDS THE X LOCATION
  setpoint_nwu << 1, 0, 0;
  nwu2enu(setpoint_nwu, setpoint_enu);
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint_enu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  setpoint_nwu << 1, 1, 0;
  nwu2enu(setpoint_nwu, setpoint_enu);
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint_enu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) < 0.0);
  ASSERT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING YAW
  setpoint_nwu << 0, 0, 0;
  nwu2enu(setpoint_nwu, setpoint_enu);
  actual.position << 0, 0, 0;
  yaw_setpoint = deg2rad(90.0);
  dt = 0.1;

  controller.reset();
  controller.calculate(setpoint_enu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(yaw_setpoint, controller.outputs(2));
}

TEST(PositionController, calculate2) {
  Vec3 setpoint_nwu, setpoint_enu, euler;
  Pose actual;
  float yaw_setpoint, dt;
  PositionController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HEADING AT 90 DEGREE
  setpoint_nwu << 1, 0, 0;  // setpoint infront of quad
  nwu2enu(setpoint_nwu, setpoint_enu);

  actual.position << 0, 0, 0;
  euler << 0.0, 0.0, deg2rad(90.0);
  euler2quat(euler, 321, actual.q);

  yaw_setpoint = 0;
  dt = 0.1;

  controller.calculate(setpoint_enu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) > 0);
  ASSERT_NEAR(0.0, controller.outputs(1), 0.1);
  ASSERT_NEAR(controller.hover_throttle, controller.outputs(3), 0.01);
}

}  // end of awesomo namepsace
