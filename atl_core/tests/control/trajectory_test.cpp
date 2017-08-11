#include "atl/control/trajectory.hpp"
#include "atl/atl_test.hpp"

#define TEST_TRAJ "tests/configs/trajectory/0.csv"

namespace atl {

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

} // end of atl namepsace
