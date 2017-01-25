#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/trajectory.hpp"

#define TEST_TRAJECTORY_OUTPUT_FILE "/tmp/trajectory.output"
#define TEST_PATH_OUTPUT_FILE "/tmp/path.output"


namespace awesomo {

static int trajectory_file_init(std::ofstream &output_file,
                                std::string output_path) {
  output_file.open(output_path);
  output_file << "time_step" << ",";
  output_file << "x" << ",";
  output_file << "z" << ",";
  output_file << "theta" << "\n";
  return 0;
}

static void record_time_step(std::ofstream &output_file, int i, VecX x) {
  output_file << i << ",";
  output_file << x(0) << ",";
  output_file << x(2) << ",";
  output_file << x(4) << "\n";
}

static int path_file_init(std::ofstream &output_file,
                                std::string output_path) {
  output_file.open(output_path);
  output_file << "x" << ",";
  output_file << "z" << "\n";
  return 0;
}

static void record_path(std::ofstream &output_file, Vec2 path) {
  output_file << path(0) << ",";
  output_file << path(1) << "\n";
}

TEST(Trajectory, quadrotor_2d_model) {
  double dt;
  VecX x(5);
  Vec2 u;
  std::ofstream output_file;

  // setup
  dt = 0.1;
  trajectory_file_init(output_file, TEST_TRAJECTORY_OUTPUT_FILE);

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

  // clean up
  output_file.close();
}

TEST(Trajectory, trajectory_desired_path) {
  Vec2 pos_init, pos_final;
  int nb_steps;
  std::vector<Vec2> desired_path;
  std::ofstream path_file;

  // setup
  pos_init << -1, 3.5;
  pos_final << 2.3, 0;
  nb_steps = 100;
  path_file_init(path_file, TEST_PATH_OUTPUT_FILE);

  // test
  desired_path = trajectory_desired_path(pos_init, pos_final, nb_steps);
  for (int i = 0; i < desired_path.size(); i++) {
    record_path(path_file, desired_path[i]);
  }

  // assert
  ASSERT_EQ(nb_steps, desired_path.size());
  ASSERT_FLOAT_EQ(pos_init(0), desired_path.front()(0));
  ASSERT_FLOAT_EQ(pos_init(1), desired_path.front()(1));
  ASSERT_FLOAT_EQ(pos_final(0), desired_path.back()(0));
  ASSERT_FLOAT_EQ(pos_final(1), desired_path.back()(1));

  // clean up
  path_file.close();
}

}  // end of awesomo namespace
