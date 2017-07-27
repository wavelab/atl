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

  EXPECT_FLOAT_EQ(0.0, controller.y_k_p);
  EXPECT_FLOAT_EQ(0.0, controller.y_k_i);
  EXPECT_FLOAT_EQ(0.0, controller.y_k_d);

  EXPECT_FLOAT_EQ(0.0, controller.z_k_p);
  EXPECT_FLOAT_EQ(0.0, controller.z_k_i);
  EXPECT_FLOAT_EQ(0.0, controller.z_k_d);

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

TEST(WaypointController, closestPoint) {
  WaypointController controller;

  // setup
  Vec3 wp_start{1.0, 1.0, 0.0};
  Vec3 wp_end{10.0, 10.0, 0.0};
  Vec3 position{5.0, 8.0, 0.0};
  Vec3 expected{6.5, 6.5, 0.0};

  // test and assert
  Vec3 point = controller.closestPoint(position, wp_start, wp_end);
  EXPECT_TRUE(point.isApprox(expected));

  std::cout << point.transpose() << std::endl;
  std::cout << expected.transpose() << std::endl;
}

TEST(WaypointController, pointLineSide) {
  WaypointController controller;

  Vec3 wp_start{0.0, 0.0, 0.0};
  Vec3 wp_end{5.0, 0.0, 0.0};

  // test colinear
  Vec3 position{0.0, 0.0, 0.0};
  double s = controller.pointLineSide(wp_start, wp_end, position);
  EXPECT_EQ(0, s);

  // test left side
  position << 0.0, 3.0, 0.0;
  s = controller.pointLineSide(wp_start, wp_end, position);
  EXPECT_EQ(1, s);

  // test right side
  position << 0.0, -3.0, 0.0;
  s = controller.pointLineSide(wp_start, wp_end, position);
  EXPECT_EQ(-1, s);
}

TEST(WaypointController, crossTrackError) {
  WaypointController controller;

  Vec3 wp_start{0.0, 0.0, 0.0};
  Vec3 wp_end{5.0, 0.0, 0.0};

  Vec3 position = Vec3::Zero();
  double e = 0.0;

  // test position to the right of waypoint track
  position << 2.0, -3.0, 1.0;
  e = controller.crossTrackError(wp_start, wp_end, position);
  EXPECT_FLOAT_EQ(-3.0, e);

  // test position to the left of waypoint track
  position << 2.0, 3.0, 1.0;
  e = controller.crossTrackError(wp_start, wp_end, position);
  EXPECT_FLOAT_EQ(3.0, e);
}

TEST(WaypointController, calculateWaypoint) {
  WaypointController controller;

  // setup
  double r = 2.0;
  Vec3 wp_start{1.0, 1.0, 0.0};
  Vec3 wp_end{10.0, 10.0, 0.0};
  Vec3 pos{5.0, 8.0, 0.0};
  Vec3 exp{6.5, 6.5, 0.0};

  // test and assert
  Vec3 point = controller.calculateWaypoint(pos, r, wp_start, wp_end);
  // EXPECT_TRUE(point.isApprox(exp));

  std::cout << point.transpose() << std::endl;
  std::cout << exp.transpose() << std::endl;
}

TEST(WaypointController, waypointHeading) {
  WaypointController controller;

  Vec3 wp_start{0.0, 0.0, 0.0};
  Vec3 wp_end{1.0, 1.0, 0.0};

  double heading = controller.waypointHeading(wp_start, wp_end);
  // EXPECT_FLOAT_EQ(
  std::cout << heading << std::endl;
}

TEST(WaypointController, waypointReached) {
  WaypointController controller;

  // setup
  double threshold = 1.0;
  Vec3 wp{5.0, 8.0, 0.0};
  Vec3 pos{5.0, 8.0, 0.0};
  controller.configured = true;

  // test and assert
  int retval = controller.waypointReached(pos, wp, threshold);
  EXPECT_EQ(1, retval);

  pos << 10.0, 10.0, 0.0;
  retval = controller.waypointReached(pos, wp, threshold);
  EXPECT_EQ(0, retval);
}

TEST(WaypointController, waypointUpdate) {
  WaypointController controller;

  // setup
  Vec3 position{2.0, 2.0, 0.0};
  Vec3 wp;

  wp << 0.0, 0.0, 0.0;
  controller.wp_start = wp;
  controller.waypoints.push_back(wp);

  wp << 10.0, 10.0, 0.0;
  controller.wp_end = wp;
  controller.waypoints.push_back(wp);

  wp << 15.0, 10.0, 0.0;
  controller.waypoints.push_back(wp);

  controller.look_ahead_dist = 1;
  controller.wp_threshold = 0.1;
  controller.configured = true;

  // test and assert
  std::ofstream outfile;
  outfile.open("update.dat");

  for (int i = 0; i < 40; i++) {
    // waypoint update
    Vec3 waypoint;
    int retval = controller.waypointUpdate(position, waypoint);
    EXPECT_EQ(0, retval);

    // record
    outfile << controller.wp_start(0) << ", ";
    outfile << controller.wp_start(1) << ", ";
    outfile << controller.wp_start(2) << std::endl;

    outfile << controller.wp_end(0) << ", ";
    outfile << controller.wp_end(1) << ", ";
    outfile << controller.wp_end(2) << std::endl;

    outfile << position(0) << ", ";
    outfile << position(1) << ", ";
    outfile << position(2) << std::endl;

    outfile << waypoint(0) << ", ";
    outfile << waypoint(1) << ", ";
    outfile << waypoint(2) << std::endl;
    outfile << std::endl;

    // update position
    position(0) = position(0) + 0.5;
    position(1) = position(1) + 0.5;
  }

  outfile.close();
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
