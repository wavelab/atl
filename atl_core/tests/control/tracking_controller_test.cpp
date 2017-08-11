#include "atl/control/tracking_controller.hpp"
#include "atl/atl_test.hpp"

#define TEST_CONFIG "tests/configs/control/tracking_controller.yaml"

namespace atl {

TEST(TrackingController, constructor) {
  TrackingController controller;

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

  EXPECT_FLOAT_EQ(0.0, controller.track_offset[0]);
  EXPECT_FLOAT_EQ(0.0, controller.track_offset[1]);
  EXPECT_FLOAT_EQ(0.0, controller.track_offset[2]);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(TrackingController, configure) {
  TrackingController controller;

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

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(1.0, controller.track_offset[0]);
  EXPECT_FLOAT_EQ(2.0, controller.track_offset[1]);
  EXPECT_FLOAT_EQ(3.0, controller.track_offset[2]);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(TrackingController, calculate) {
  Vec3 errors;
  double yaw, dt;
  TrackingController controller;

  // setup
  controller.configure(TEST_CONFIG);
  controller.track_offset << 0.0, 0.0, 0.0;

  // CHECK HOVERING PID OUTPUT
  errors << 0, 0, 0;
  yaw = 0.0;
  dt = 0.1;
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(controller.hover_throttle, controller.outputs(3));

  // CHECK MOVING TOWARDS THE X LOCATION
  errors << 1, 0, 0;
  yaw = 0.0;
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING TOWARDS THE Y LOCATION
  errors << 0, 1, 0;
  yaw = 0.0;
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  EXPECT_TRUE(controller.outputs(0) < 0.0);
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  errors << 1, 1, 0;
  yaw = 0.0;
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  EXPECT_TRUE(controller.outputs(0) < 0.0);
  EXPECT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING YAW
  errors << 0, 0, 0;
  yaw = deg2rad(90.0);
  dt = 0.1;

  controller.reset();
  controller.calculate(errors, yaw, dt);
  controller.printOutputs();

  EXPECT_FLOAT_EQ(yaw, controller.outputs(2));
}

} // end of atl namepsace
