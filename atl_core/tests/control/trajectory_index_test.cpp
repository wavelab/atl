#include "atl/control/trajectory_index.hpp"
#include "atl/atl_test.hpp"

#define TEST_CONFIG "tests/configs/control/landing_controller.yaml"
#define TEST_TRAJ_INDEX "tests/configs/trajectory/index.csv"

namespace atl {

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

} // end of atl namepsace
