#include "atl/atl_test.hpp"
#include "atl/control/landing_controller.hpp"

#define TEST_CONFIG "tests/configs/control/landing_controller.yaml"

namespace atl {

TEST(LandingController, constructor) {
  LandingController controller;

  EXPECT_FALSE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);

  EXPECT_FLOAT_EQ(0.0, controller.x_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.x_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.x_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.y_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.y_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.y_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.vz_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.vz_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.vz_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.hover_throttle);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(LandingController, configure) {
  LandingController controller;

  controller.configure(TEST_CONFIG);

  EXPECT_TRUE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);

  EXPECT_FLOAT_EQ(1.0, controller.x_controller.k_p);
  EXPECT_FLOAT_EQ(2.0, controller.x_controller.k_i);
  EXPECT_FLOAT_EQ(3.0, controller.x_controller.k_d);

  EXPECT_FLOAT_EQ(1.0, controller.y_controller.k_p);
  EXPECT_FLOAT_EQ(2.0, controller.y_controller.k_i);
  EXPECT_FLOAT_EQ(3.0, controller.y_controller.k_d);

  EXPECT_FLOAT_EQ(1.0, controller.vz_controller.k_p);
  EXPECT_FLOAT_EQ(2.0, controller.vz_controller.k_i);
  EXPECT_FLOAT_EQ(3.0, controller.vz_controller.k_d);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.5, controller.hover_throttle);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

} // end of atl namepsace
