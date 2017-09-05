#include "atl/control/trajectory_controller.hpp"
#include "atl/atl_test.hpp"

#define TEST_CONFIG "tests/configs/control/trajectory_controller.yaml"

namespace atl {

TEST(TrajectoryController, constructor) {
  TrajectoryController controller;

  EXPECT_FALSE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);
  EXPECT_FLOAT_EQ(0.0, controller.blackbox_dt);

  // EXPECT_FLOAT_EQ(0.0, controller.vx_controller.k_p);
  // EXPECT_FLOAT_EQ(0.0, controller.vx_controller.k_i);
  // EXPECT_FLOAT_EQ(0.0, controller.vx_controller.k_d);
  //
  // EXPECT_FLOAT_EQ(0.0, controller.vy_controller.k_p);
  // EXPECT_FLOAT_EQ(0.0, controller.vy_controller.k_i);
  // EXPECT_FLOAT_EQ(0.0, controller.vy_controller.k_d);
  //
  // EXPECT_FLOAT_EQ(0.0, controller.vz_controller.k_p);
  // EXPECT_FLOAT_EQ(0.0, controller.vz_controller.k_i);
  // EXPECT_FLOAT_EQ(0.0, controller.vz_controller.k_d);

  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));

  EXPECT_FLOAT_EQ(1.0, controller.trajectory_threshold(0));
  EXPECT_FLOAT_EQ(1.0, controller.trajectory_threshold(1));
  EXPECT_FLOAT_EQ(1.0, controller.trajectory_threshold(2));

  EXPECT_FALSE(controller.blackbox_enable);
  EXPECT_FLOAT_EQ(FLT_MAX, controller.blackbox_rate);
}

TEST(TrajectoryController, configure) {
  TrajectoryController controller;

  controller.configure(TEST_CONFIG);

  EXPECT_TRUE(controller.configured);

  EXPECT_FLOAT_EQ(0.0, controller.dt);
  EXPECT_FLOAT_EQ(0.0, controller.blackbox_dt);

  // EXPECT_FLOAT_EQ(1.0, controller.vx_controller.k_p);
  // EXPECT_FLOAT_EQ(2.0, controller.vx_controller.k_i);
  // EXPECT_FLOAT_EQ(3.0, controller.vx_controller.k_d);
  //
  // EXPECT_FLOAT_EQ(1.0, controller.vy_controller.k_p);
  // EXPECT_FLOAT_EQ(2.0, controller.vy_controller.k_i);
  // EXPECT_FLOAT_EQ(3.0, controller.vy_controller.k_d);
  //
  // EXPECT_FLOAT_EQ(1.0, controller.vz_controller.k_p);
  // EXPECT_FLOAT_EQ(2.0, controller.vz_controller.k_i);
  // EXPECT_FLOAT_EQ(3.0, controller.vz_controller.k_d);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.roll_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.roll_limit[1]);

  EXPECT_FLOAT_EQ(deg2rad(-20.0), controller.pitch_limit[0]);
  EXPECT_FLOAT_EQ(deg2rad(20.0), controller.pitch_limit[1]);

  EXPECT_FLOAT_EQ(-1.0, controller.throttle_limit[0]);
  EXPECT_FLOAT_EQ(1.0, controller.throttle_limit[1]);

  EXPECT_FLOAT_EQ(0.0, controller.setpoints(0));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(1));
  EXPECT_FLOAT_EQ(0.0, controller.setpoints(2));

  EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(2));
  EXPECT_FLOAT_EQ(0.0, controller.outputs(3));

  EXPECT_FLOAT_EQ(0.5, controller.trajectory_threshold(0));
  EXPECT_FLOAT_EQ(0.5, controller.trajectory_threshold(1));
  EXPECT_FLOAT_EQ(0.5, controller.trajectory_threshold(2));

  EXPECT_TRUE(controller.blackbox_enable);
  EXPECT_FLOAT_EQ(0.2, controller.blackbox_rate);
}

TEST(TrajectoryController, loadTrajectory) {
  TrajectoryController controller;
  Vec3 p0, pf;
  double v;
  int retval;

  p0 << 0, 0, 5;
  pf << 5, 0, 0;
  v = 0;

  controller.configure(TEST_CONFIG);
  retval = controller.loadTrajectory(p0, pf, v);

  EXPECT_EQ(0, retval);
  EXPECT_TRUE(controller.trajectory.loaded);
  EXPECT_EQ(50, controller.trajectory.pos.size());
  EXPECT_EQ(50, controller.trajectory.vel.size());
  EXPECT_EQ(50, controller.trajectory.inputs.size());
}

// TEST(TrajectoryController, calculateVelocityErrors) {
//   Vec3 v_errors;
//   double dt;
//   TrajectoryController controller;
//
//   // setup
//   controller.configure(TEST_CONFIG);
//
//   // CHECK HOVERING PID OUTPUT
//   v_errors << 0, 0, 0;
//   dt = 0.1;
//
//   controller.calculateVelocityErrors(v_errors, 0.0, dt);
//   controller.printOutputs();
//
//   EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
//   EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
//   EXPECT_FLOAT_EQ(0.0, controller.outputs(3));
//
//   // CHECK MOVING TOWARDS THE X LOCATION
//   v_errors << 1, 0, 0;
//   dt = 0.1;
//
//   controller.reset();
//   controller.calculateVelocityErrors(v_errors, 0.0, dt);
//   controller.printOutputs();
//
//   EXPECT_FLOAT_EQ(0.0, controller.outputs(0));
//   EXPECT_TRUE(controller.outputs(1) > 0.0);
//
//   // CHECK MOVING TOWARDS THE Y LOCATION
//   v_errors << 0, 1, 0;
//   dt = 0.1;
//
//   controller.reset();
//   controller.calculateVelocityErrors(v_errors, 0.0, dt);
//   controller.printOutputs();
//
//   EXPECT_TRUE(controller.outputs(0) < 0.0);
//   EXPECT_FLOAT_EQ(0.0, controller.outputs(1));
//
//   // CHECK MOVING TOWARDS THE X AND Y LOCATION
//   v_errors << 1, 1, 0;
//   dt = 0.1;
//
//   controller.reset();
//   controller.calculateVelocityErrors(v_errors, 0.0, dt);
//   controller.printOutputs();
//
//   EXPECT_TRUE(controller.outputs(0) < 0.0);
//   EXPECT_TRUE(controller.outputs(1) > 0.0);
// }

} // end of atl namepsace
