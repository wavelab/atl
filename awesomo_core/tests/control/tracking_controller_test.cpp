#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/control/tracking_controller.hpp"

#define TEST_CONFIG "tests/configs/control/tracking_controller.yaml"


namespace awesomo {

TEST(TrackingController, constructor) {
  TrackingController controller;

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

TEST(TrackingController, configure) {
  TrackingController controller;

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

TEST(TrackingController, calculate) {
  Vec3 errors;
  double yaw, dt;
  TrackingController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HOVERING PID OUTPUT
  errors << 0, 0, 0;
  yaw = 0.0;
  dt = 0.1;
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(controller.hover_throttle, controller.outputs(3));

  // CHECK MOVING TOWARDS THE Y LOCATION
  errors << 0, 1, 0;
  yaw = 0.0;
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) < 0.0);
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));

  // CHECK MOVING TOWARDS THE X LOCATION
  errors << 1, 0, 0;
  yaw = 0.0;
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  errors << 1, 1, 0;
  yaw = 0.0;
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) < 0.0);
  ASSERT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING YAW
  errors << 0, 0, 0;
  yaw = deg2rad(90.0);
  dt = 0.1;

  controller.reset();
  Vec4 outputs = controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(yaw, outputs(2));
}

}  // end of awesomo namepsace
