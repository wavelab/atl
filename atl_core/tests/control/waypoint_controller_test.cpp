#include "atl/control/waypoint_controller.hpp"
#include "atl/atl_test.hpp"

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
  EXPECT_FLOAT_EQ(0.0, controller.hover_throttle);

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
  Mission mission;
  WaypointController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // push waypoints in ENU coorindates
  // note that the controller works in NWU coordinates
  Vec3 wp;
  wp << 0.0, 0.0, 10.0;
  mission.wp_start = wp;
  mission.local_waypoints.push_back(wp);

  wp << 10.0, 6.0, 10.0;
  mission.wp_end = wp;
  mission.local_waypoints.push_back(wp);

  wp << 17.0, -7.0, 10.0;
  mission.local_waypoints.push_back(wp);

  wp << 9.0, -10.0, 10.0;
  mission.local_waypoints.push_back(wp);
  mission.configured = true;

  // update controller
  Pose pose{WORLD_FRAME, 0.0, 0.0, deg2rad(-90.0), 0.0, 0.0, 10.0};
  Vec3 vel{1.0, 0.0, 0.0};
  double dt = 0.011;
  controller.update(mission, pose, vel, dt);

  // Pose pose2{0.0, 0.0, deg2rad(0.0), 5.0, 6.0, 10.0};
  // controller.update(mission, pose2, vel, dt);
  //
  // Pose pose3{0.0, 0.0, deg2rad(0.0), 5.0, 6.0, 10.0};
  // controller.update(mission, pose3, vel, dt);
}

} // end of atl namepsace
