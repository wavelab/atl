#include "atl/control/landing_controller.hpp"
#include "atl/atl_test.hpp"

#define TEST_CONFIG "tests/configs/control/landing_controller.yaml"
#define TEST_TRAJ_INDEX "tests/configs/trajectory/index.csv"
#define TEST_TRAJ "tests/configs/trajectory/0.csv"

namespace atl {

// TRAJECTORY
TEST(Trajectory, constructor) {
  Trajectory trajectory;

  EXPECT_FALSE(trajectory.loaded);
}

TEST(Trajectory, load) {
  int retval;
  Trajectory traj;
  Vec3 pos;

  pos << 1.0, 2.0, 3.0;
  retval = traj.load(1, TEST_TRAJ, pos);
  EXPECT_EQ(0, retval);
  EXPECT_EQ(1, traj.index);
  EXPECT_FLOAT_EQ(1.0, traj.p0(0));
  EXPECT_FLOAT_EQ(2.0, traj.p0(1));
  EXPECT_FLOAT_EQ(3.0, traj.p0(2));
  EXPECT_EQ(50, traj.pos.size());
  EXPECT_EQ(50, traj.vel.size());
  EXPECT_EQ(50, traj.inputs.size());
  EXPECT_EQ(50, traj.rel_pos.size());
  EXPECT_EQ(50, traj.rel_vel.size());
}

TEST(Trajectory, update) {
  Trajectory traj;
  Vec3 pos;
  Vec2 wp_pos, wp_vel, wp_inputs;

  pos << 0.0, 0.0, 5.0;
  traj.load(1, TEST_TRAJ, pos);

  // pos << 0.0, 0.0, 5.0;
  // traj.update(pos, wp_pos, wp_vel, wp_inputs);
  // EXPECT_FLOAT_EQ(0.0, wp_pos(0));
  // EXPECT_FLOAT_EQ(5.0, wp_pos(1));
  // EXPECT_FLOAT_EQ(0.0, wp_vel(0));
}

TEST(Trajectory, reset) {
  Trajectory traj;
  Vec3 pos;

  pos << 1.0, 2.0, 3.0;
  traj.load(1, TEST_TRAJ, pos);
  traj.reset();
  EXPECT_EQ(0, traj.pos.size());
  EXPECT_EQ(0, traj.vel.size());
  EXPECT_EQ(0, traj.inputs.size());
}

// TRAJECTORY INDEX
TEST(TrajectoryIndex, constructor) {
  TrajectoryIndex index;

  EXPECT_FALSE(index.loaded);
}

TEST(TrajectoryIndex, load) {
  int retval;
  TrajectoryIndex index;

  retval = index.load(TEST_TRAJ_INDEX);
  EXPECT_EQ(0, retval);
  EXPECT_TRUE(index.loaded);
  EXPECT_EQ(1, index.index_data.rows());
  EXPECT_EQ(3, index.index_data.cols());

  EXPECT_FLOAT_EQ(0.2, index.pos_thres);
  EXPECT_FLOAT_EQ(0.2, index.vel_thres);
}

TEST(TrajectoryIndex, find) {
  int retval;
  Vec3 pos;
  double v;
  Trajectory traj;
  TrajectoryIndex index;

  // setup
  index.load(TEST_TRAJ_INDEX);

  // test and assert
  pos << 0.0, 0.0, 5.0;
  v = 0.0;
  retval = index.find(pos, v, traj);

  EXPECT_EQ(0, retval);
  EXPECT_EQ(50, traj.pos.size());
  EXPECT_EQ(50, traj.vel.size());
  EXPECT_EQ(50, traj.inputs.size());
  EXPECT_EQ(50, traj.rel_pos.size());
  EXPECT_EQ(50, traj.rel_vel.size());
}

// LANDING CONTROLLER
TEST(LandingController, constructor) {
  LandingController controller;

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

TEST(LandingController, configure) {
  LandingController controller;

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

TEST(LandingController, loadTrajectory) {
  LandingController controller;
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

// TEST(LandingController, calculateVelocityErrors) {
//   Vec3 v_errors;
//   double dt;
//   LandingController controller;
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

}  // end of atl namepsace
