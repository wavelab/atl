#include "atl/atl_test.hpp"
#include "atl/control/waypoint_controller.hpp"

#define TEST_CONFIG "tests/configs/control/waypoint_controller.yaml"

namespace atl {

TEST(WaypointController, constructor) {
  WaypointController controller;

  EXPECT_FALSE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);

  EXPECT_FLOAT_EQ(0.0, controller.at_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.at_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.at_controller.k_d);
  EXPECT_FLOAT_EQ(0.0, controller.ct_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.ct_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.ct_controller.k_d);
  EXPECT_FLOAT_EQ(0.0, controller.z_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.z_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.z_controller.k_d);
  EXPECT_FLOAT_EQ(0.0, controller.yaw_controller.k_p);
  EXPECT_FLOAT_EQ(0.0, controller.yaw_controller.k_i);
  EXPECT_FLOAT_EQ(0.0, controller.yaw_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[1]);
  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[1]);
  EXPECT_FLOAT_EQ(0.5, controller.hover_throttle);

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(WaypointController, configure) {
  WaypointController controller;

  controller.configure(TEST_CONFIG);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}

TEST(WaypointController, update) {
  WaypointController controller;

  Vec3 pos_setpoints{1.0, 0.0, 0.0};
  Vec3 vel_setpoints{0.0, 0.0, 0.0};
  Pose pose{0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  Vec3 vel{0.0, 0.0, 0.0};

  controller.wp_start = Vec3{0.0, 0.0, 0.0};
  controller.wp_end = Vec3{0.0, 5.0, 0.0};

  // controller.configure(TEST_CONFIG);
  // controller.update(pose, vel, dt);
  //   pos_setpoints, vel_setpoints, yaw_setpoint, pose, vel, dt);

  // std::cout << controller.outputs.transpose() << std::endl;
}


}  // end of atl namepsace
