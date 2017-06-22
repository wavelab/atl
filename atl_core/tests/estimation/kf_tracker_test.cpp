#include <random>
#include <fstream>
#include <iostream>

#include "atl_core/atl_test.hpp"
#include "atl_core/estimation/kf_tracker.hpp"

#define TEST_CONFIG "tests/configs/estimation/kf_tracker.yaml"
#define TEST_OUTPUT_FILE "/tmp/estimation_kf_tracker_test.output"


namespace atl {

static int prepareOutputFile(std::ofstream &output_file,
                             std::string output_path) {
  // clang-format off
  output_file.open(output_path);

  output_file << "time_step" << ",";

  output_file << "x" << ",";
  output_file << "y" << ",";
  output_file << "z" << ",";

  output_file << "bx" << ",";
  output_file << "by" << ",";
  output_file << "bz" << std::endl;
  // clang-format on

  return 0;
}

static void recordTimeStep(std::ofstream &output_file,
                           int i,
                           Vec3 mea,
                           Vec3 est) {
  // record true state x, y, z
  output_file << i << ",";
  output_file << mea(0) << ",";
  output_file << mea(1) << ",";
  output_file << mea(2) << ",";

  // record belief state x, y, z
  output_file << est(0) << ",";
  output_file << est(1) << ",";
  output_file << est(2) << std::endl;
}

TEST(KalmanFilterTracker, sanityCheck) {
  int retval;
  KalmanFilterTracker tracker;
  Vec3 prev_pos, curr_pos;

  tracker.configure(TEST_CONFIG);
  tracker.initialized = true;

  prev_pos << 0, 0, 0;
  curr_pos << 0, 0, 5;
  retval = tracker.sanityCheck(prev_pos, curr_pos);
  ASSERT_EQ(0, retval);

  prev_pos << 0, 0, 0;
  curr_pos << 10, 20, 30;
  retval = tracker.sanityCheck(prev_pos, curr_pos);
  ASSERT_EQ(-2, retval);
}

TEST(KalmanFilterTracker, estimate) {
  float dt;
  KalmanFilterTracker tracker;
  MatX A(9, 9);
  Vec3 pos, vel, acc, mea, est;
  VecX state(9), mu(9), y(3), motion_noise(3);
  std::ofstream output_file;
  std::default_random_engine rgen;
  std::normal_distribution<float> norm_x(0, 0.5);
  std::normal_distribution<float> norm_y(0, 0.5);
  std::normal_distribution<float> norm_z(0, 0.5);

  // setup
  // clang-format off
  dt = 0.1;
  pos << 0, 0, 0;
  vel << 9, 30, 0;
  acc << 0, -10, 0;
  mu << 0.0, 0.0, 0.0,    // x, y, z
        9.0, 30.0, 0.0,   // x_dot, y_dot, z_dot
        0.0, -10.0, 0.0;  // x_ddot, y_ddot, z_ddot
  tracker.configure(TEST_CONFIG);
  tracker.initialize(mu);
  prepareOutputFile(output_file, TEST_OUTPUT_FILE);
  // clang-format on

  // estimate
  for (int i = 0; i < 20; i++) {
    // update true state
    // clang-format off
    vel = vel + acc * dt;
    pos = pos + vel * dt;
    state << pos(0), pos(1), pos(2),
             vel(0), vel(1), vel(2),
             acc(0), acc(1), acc(2);
    // clang-format on

    // perform measurement
    motion_noise << norm_x(rgen), norm_y(rgen), norm_z(rgen);
    y = tracker.C * state + motion_noise;

    // estimate
    MATRIX_A_CONSTANT_ACCELERATION_XYZ(A);
    tracker.estimate(A, y);

    // record
    mea << pos(0), pos(1), pos(2);
    est << tracker.mu(0), tracker.mu(1), tracker.mu(2);
    recordTimeStep(output_file, i, mea, est);
  }
  output_file.close();
}

}  // end of atl namespace
