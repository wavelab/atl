#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/trajectory.hpp"

#define TEST_TRAJECTORY_OUTPUT_FILE "/tmp/trajectory.output"


namespace awesomo {

static int setup_output_file(std::ofstream &output_file,
                             std::string output_path) {
  output_file.open(output_path);

  output_file << "time_step" << ",";
  output_file << "x" << ",";
  output_file << "z" << ",";
  output_file << "theta" << "\n";

  return 0;
}

static void record_time_step(std::ofstream &output_file,
                             int i,
                             VecX x) {
  // record true state x, z
  output_file << i << ",";
  output_file << x(0) << ",";
  output_file << x(2) << ",";
  output_file << x(4) << "\n";
}

TEST(Trajectory, quadrotor_2d_model) {
  double dt;
  VecX x(5);
  Vec2 u;
  std::ofstream output_file;

  // setup
  dt = 0.1;
  setup_output_file(output_file, TEST_TRAJECTORY_OUTPUT_FILE);

  // initial state
  x(0) = 0.0;
  x(1) = 0.0;
  x(2) = 0.0;
  x(3) = 0.0;
  x(4) = 0.0;

  // initial inputs
  u(0) = 9.85;
  u(1) = 0.0;

  for (int i = 1; i < 10; i++) {
    x = quadrotor_2d_model(x, u);
    record_time_step(output_file, i, x);
  }
}

}  // end of awesomo namespace
