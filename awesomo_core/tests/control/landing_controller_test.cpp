#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/control/landing_controller.hpp"

#define TEST_CONFIG "tests/configs/control/landing_controller.yaml"
#define TEST_TRAJ_INDEX "tests/configs/trajectory/index.csv"
#define TEST_TRAJ "tests/configs/trajectory/0.csv"


namespace awesomo {

// TRAJECTORY
TEST(Trajectory, constructor) {
  Trajectory trajectory;

  ASSERT_FALSE(trajectory.loaded);
}

TEST(Trajectory, load) {
  int retval;
  Trajectory traj;
  Vec3 pos;

  pos << 1.0, 2.0, 3.0;
  retval = traj.load(TEST_TRAJ, pos);
  ASSERT_EQ(0, retval);
  ASSERT_FLOAT_EQ(1.0, traj.p0(0));
  ASSERT_FLOAT_EQ(2.0, traj.p0(1));
  ASSERT_FLOAT_EQ(3.0, traj.p0(2));
  ASSERT_EQ(50, traj.pos.size());
  ASSERT_EQ(50, traj.vel.size());
  ASSERT_EQ(50, traj.inputs.size());
  ASSERT_EQ(50, traj.rel_pos.size());
  ASSERT_EQ(50, traj.rel_vel.size());
}

TEST(Trajectory, update) {
  Trajectory traj;
  Vec3 pos;
  Vec2 wp_pos, wp_vel, wp_inputs;

  pos << 0.0, 0.0, 5.0;
  traj.load(TEST_TRAJ, pos);

  pos << 0.0, 0.0, 5.0;
  traj.update(pos, wp_pos, wp_vel, wp_inputs);
  ASSERT_FLOAT_EQ(0.0, wp_pos(0));
  ASSERT_FLOAT_EQ(5.0, wp_pos(1));
  ASSERT_FLOAT_EQ(0.0, wp_vel(0));
  ASSERT_EQ(49, traj.pos.size());

  pos << 0.0, 0.0, 4.9;
  traj.update(pos, wp_pos, wp_vel, wp_inputs);
  ASSERT_EQ(48, traj.pos.size());

  pos << 0.0, 0.0, 4.8;
  traj.update(pos, wp_pos, wp_vel, wp_inputs);
  ASSERT_EQ(47, traj.pos.size());
}

TEST(Trajectory, reset) {
  Trajectory traj;
  Vec3 pos;

  pos << 1.0, 2.0, 3.0;
  traj.load(TEST_TRAJ, pos);
  traj.reset();
  ASSERT_EQ(0, traj.pos.size());
  ASSERT_EQ(0, traj.vel.size());
  ASSERT_EQ(0, traj.inputs.size());
}


// TRAJECTORY INDEX
TEST(TrajectoryIndex, constructor) {
  TrajectoryIndex index;

  ASSERT_FALSE(index.loaded);
}

TEST(TrajectoryIndex, load) {
  int retval;
  TrajectoryIndex index;

  retval = index.load(TEST_TRAJ_INDEX);
  ASSERT_EQ(0, retval);
  ASSERT_TRUE(index.loaded);
  ASSERT_EQ(1, index.index_data.rows());
  ASSERT_EQ(3, index.index_data.cols());

  ASSERT_FLOAT_EQ(0.2, index.pos_thres);
  ASSERT_FLOAT_EQ(0.2, index.vel_thres);
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

  ASSERT_EQ(0, retval);
  ASSERT_EQ(50, traj.pos.size());
  ASSERT_EQ(50, traj.vel.size());
  ASSERT_EQ(50, traj.inputs.size());
  ASSERT_EQ(50, traj.rel_pos.size());
  ASSERT_EQ(50, traj.rel_vel.size());
}


// LANDING CONTROLLER
TEST(LandingController, constructor) {
  LandingController controller;

  ASSERT_FALSE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.dt);
  ASSERT_FLOAT_EQ(0.0, controller.blackbox_dt);

  ASSERT_FLOAT_EQ(0.0, controller.vx_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.vx_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.vx_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.vy_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.vy_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.vy_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.vz_controller.k_p);
  ASSERT_FLOAT_EQ(0.0, controller.vz_controller.k_i);
  ASSERT_FLOAT_EQ(0.0, controller.vz_controller.k_d);

  ASSERT_FLOAT_EQ(0.0, controller.roll_limit[0]);
  ASSERT_FLOAT_EQ(0.0, controller.roll_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.pitch_limit[0]);
  ASSERT_FLOAT_EQ(0.0, controller.pitch_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.setpoints(0));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(0));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(1));

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(2));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(3));

  ASSERT_FLOAT_EQ(1.0, controller.trajectory_threshold(0));
  ASSERT_FLOAT_EQ(1.0, controller.trajectory_threshold(1));
  ASSERT_FLOAT_EQ(1.0, controller.trajectory_threshold(2));

  ASSERT_FALSE(controller.blackbox_enable);
  ASSERT_FLOAT_EQ(FLT_MAX, controller.blackbox_rate);
}

TEST(LandingController, configure) {
  LandingController controller;

  controller.configure(TEST_CONFIG);

  ASSERT_TRUE(controller.configured);

  ASSERT_FLOAT_EQ(0.0, controller.dt);
  ASSERT_FLOAT_EQ(0.0, controller.blackbox_dt);

  ASSERT_FLOAT_EQ(1.0, controller.vx_controller.k_p);
  ASSERT_FLOAT_EQ(2.0, controller.vx_controller.k_i);
  ASSERT_FLOAT_EQ(3.0, controller.vx_controller.k_d);

  ASSERT_FLOAT_EQ(1.0, controller.vy_controller.k_p);
  ASSERT_FLOAT_EQ(2.0, controller.vy_controller.k_i);
  ASSERT_FLOAT_EQ(3.0, controller.vy_controller.k_d);

  ASSERT_FLOAT_EQ(1.0, controller.vz_controller.k_p);
  ASSERT_FLOAT_EQ(2.0, controller.vz_controller.k_i);
  ASSERT_FLOAT_EQ(3.0, controller.vz_controller.k_d);

  ASSERT_FLOAT_EQ(deg2rad(-20.0), controller.roll_limit[0]);
  ASSERT_FLOAT_EQ(deg2rad(20.0), controller.roll_limit[1]);

  ASSERT_FLOAT_EQ(deg2rad(-20.0), controller.pitch_limit[0]);
  ASSERT_FLOAT_EQ(deg2rad(20.0), controller.pitch_limit[1]);

  ASSERT_FLOAT_EQ(-1.0, controller.throttle_limit[0]);
  ASSERT_FLOAT_EQ(1.0, controller.throttle_limit[1]);

  ASSERT_FLOAT_EQ(0.0, controller.setpoints(0));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(1));
  ASSERT_FLOAT_EQ(0.0, controller.setpoints(2));

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(2));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(3));

  ASSERT_FLOAT_EQ(0.5, controller.trajectory_threshold(0));
  ASSERT_FLOAT_EQ(0.5, controller.trajectory_threshold(1));
  ASSERT_FLOAT_EQ(0.5, controller.trajectory_threshold(2));

  ASSERT_TRUE(controller.blackbox_enable);
  ASSERT_FLOAT_EQ(0.2, controller.blackbox_rate);
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

  ASSERT_EQ(0, retval);
  ASSERT_TRUE(controller.trajectory.loaded);
  ASSERT_EQ(50, controller.trajectory.pos.size());
  ASSERT_EQ(50, controller.trajectory.vel.size());
  ASSERT_EQ(50, controller.trajectory.inputs.size());
}

TEST(LandingController, calculateVelocityErrors) {
  Vec3 v_errors;
  double dt;
  LandingController controller;

  // setup
  controller.configure(TEST_CONFIG);

  // CHECK HOVERING PID OUTPUT
  v_errors << 0, 0, 0;
  dt = 0.1;

  controller.calculateVelocityErrors(v_errors, 0.0, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));
  ASSERT_FLOAT_EQ(0.0, controller.outputs(3));

  // CHECK MOVING TOWARDS THE X LOCATION
  v_errors << 1, 0, 0;
  dt = 0.1;

  controller.reset();
  controller.calculateVelocityErrors(v_errors, 0.0, dt);
  controller.printOutputs();

  ASSERT_FLOAT_EQ(0.0, controller.outputs(0));
  ASSERT_TRUE(controller.outputs(1) > 0.0);

  // CHECK MOVING TOWARDS THE Y LOCATION
  v_errors << 0, 1, 0;
  dt = 0.1;

  controller.reset();
  controller.calculateVelocityErrors(v_errors, 0.0, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) < 0.0);
  ASSERT_FLOAT_EQ(0.0, controller.outputs(1));

  // CHECK MOVING TOWARDS THE X AND Y LOCATION
  v_errors << 1, 1, 0;
  dt = 0.1;

  controller.reset();
  controller.calculateVelocityErrors(v_errors, 0.0, dt);
  controller.printOutputs();

  ASSERT_TRUE(controller.outputs(0) < 0.0);
  ASSERT_TRUE(controller.outputs(1) > 0.0);
}

}  // end of awesomo namepsace
