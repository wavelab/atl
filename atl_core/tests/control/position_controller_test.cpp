#include "atl/control/position_controller.hpp"
#include "atl/atl_test.hpp"

#define TEST_CONFIG "tests/configs/control/position_controller.yaml"

namespace atl {

TEST(PositionController, constructor) {
  PositionController controller;

  EXPECT_FALSE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);

  EXPECT_FLOAT_EQ(0.0, controller.x_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.x_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.x_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.y_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.y_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.y_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.z_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.z_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.z_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.hover_throttle);

  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(PositionController, configure) {
  PositionController controller;

  controller.configure(TEST_CONFIG);

  EXPECT_TRUE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);

  EXPECT_FLOAT_EQ(0.1, controller.x_controller.k_p);
  EXPECT_FLOAT_EQ(0.2, controller.x_controller.k_i);
  EXPECT_FLOAT_EQ(0.3, controller.x_controller.k_d);

  EXPECT_FLOAT_EQ(0.1, controller.y_controller.k_p);
  EXPECT_FLOAT_EQ(0.2, controller.y_controller.k_i);
  EXPECT_FLOAT_EQ(0.3, controller.y_controller.k_d);

  EXPECT_FLOAT_EQ(0.1, controller.z_controller.k_p);
  EXPECT_FLOAT_EQ(0.2, controller.z_controller.k_i);
  EXPECT_FLOAT_EQ(0.3, controller.z_controller.k_d);

  EXPECT_FLOAT_EQ(0.6, controller.hover_throttle);

  EXPECT_FLOAT_EQ(deg2rad(-50.0), controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(50.0), controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(deg2rad(-50.0), controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(50.0), controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(PositionController, update) {
  Vec3 setpoint_nwu;
  Pose actual;
  float yaw_setpoint, dt;
  PositionController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HOVERING PID OUTPUT
  setpoint_nwu << 0, 0, 0;
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;
  controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(controller.hover_throttle, controller.outputs(3));

  // CHECK MOVING TOWARDS THE Y LOCATION
  setpoint_nwu << 0, 1, 0;
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  EXPECT_TRUE(controller.outputs(0) < 0.0);
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));

  // CHECK MOVING TOWARDS THE X LOCATION
  setpoint_nwu << 1, 0, 0;
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  setpoint_nwu << 1, 1, 0;
  actual.position << 0, 0, 0;
  yaw_setpoint = 0;
  dt = 0.1;

  controller.reset();
  controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  EXPECT_TRUE(controller.outputs(0) < 0.0);
  EXPECT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING YAW
  setpoint_nwu << 0, 0, 0;
  actual.position << 0, 0, 0;
  yaw_setpoint = deg2rad(90.0);
  dt = 0.1;

  controller.reset();
  controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  EXPECT_FLOAT_EQ(yaw_setpoint, controller.outputs(2));
}

TEST(PositionController, update2) {
  Vec3 setpoint_nwu, euler;
  Pose actual;
  float yaw_setpoint, dt;
  PositionController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HEADING AT 90 DEGREE
  setpoint_nwu << 1, 0, 0; // setpoint infront of quad
  actual.position << 0, 0, 0;
  euler << 0.0, 0.0, deg2rad(90.0);
  actual.orientation = euler321ToQuat(euler);

  yaw_setpoint = 0;
  dt = 0.1;

  controller.update(setpoint_nwu, actual, yaw_setpoint, dt);
  controller.printOutputs();

  EXPECT_TRUE(controller.outputs(0) > 0);
  ASSERT_NEAR(0.0, controller.outputs(1), 0.1);
  ASSERT_NEAR(controller.hover_throttle, controller.outputs(3), 0.01);
}

} // end of atl namepsace
