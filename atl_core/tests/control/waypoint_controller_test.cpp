#include "atl/atl_test.hpp"
#include "atl/control/waypoint_controller.hpp"

#define TEST_CONFIG "tests/configs/control/waypoint_controller.yaml"

namespace atl {

TEST(WaypointController, constructor) {
  WaypointController controller;

  EXPECT_FALSE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);

  EXPECT_FLOAT_EQ(0.0, controller.vx_k_p);
  EXPECT_FLOAT_EQ(0.0, controller.vx_k_i);
  EXPECT_FLOAT_EQ(0.0, controller.vx_k_d);

  EXPECT_FLOAT_EQ(0.0, controller.vy_k_p);
  EXPECT_FLOAT_EQ(0.0, controller.vy_k_i);
  EXPECT_FLOAT_EQ(0.0, controller.vy_k_d);

  EXPECT_FLOAT_EQ(0.0, controller.vz_k_p);
  EXPECT_FLOAT_EQ(0.0, controller.vz_k_i);
  EXPECT_FLOAT_EQ(0.0, controller.vz_k_d);

  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.throttle_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.throttle_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.pos_setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.pos_setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.pos_setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.vel_setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.vel_setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.vel_setpoints(2));

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

  EXPECT_FLOAT_EQ(-1.0, controller.throttle_limit[0]);
  EXPECT_FLOAT_EQ(1.0, controller.throttle_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.pos_setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.pos_setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.pos_setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.vel_setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.vel_setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.vel_setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
}


TEST(WaypointController, calculate) {
  WaypointController controller;

  Vec3 pos_setpoints{1.0, 0.0, 0.0};
  Vec3 vel_setpoints{0.0, 0.0, 0.0};
  double yaw_setpoint = 0.0;
  Pose pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Vec3 vel{0.0, 0.0, 0.0};
  double dt = 0.01;

  controller.configure(TEST_CONFIG);
  controller.calculate(
    pos_setpoints, vel_setpoints, yaw_setpoint, pose, vel, dt);

  std::cout << controller.outputs.transpose() << std::endl;
}


}  // end of atl namepsace
