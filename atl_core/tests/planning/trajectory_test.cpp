#include "atl/planning/trajectory.hpp"
#include "atl/atl_test.hpp"
#include <nlopt.hpp>

#define TEST_TRAJECTORY_OUTPUT_FILE "/tmp/trajectory.output"
#define TEST_OPTIMIZED_OUTPUT_FILE "/tmp/trajectory_optimized.output"
#define TEST_PATH_OUTPUT_FILE "/tmp/path.output"

namespace atl {

static int trajectory_file_init(std::ofstream &output_file,
                                std::string output_path) {
  // clang-format off
  output_file.open(output_path);
  output_file << "time_step" << ",";
  output_file << "x" << ",";
  output_file << "z" << ",";
  output_file << "theta" << "\n";
  // clang-format on
  return 0;
}

static void record_time_step(std::ofstream &output_file, int i, VecX x) {
  output_file << i << ",";
  output_file << x(0) << ",";
  output_file << x(2) << ",";
  output_file << x(4) << "\n";
}

static int path_file_init(std::ofstream &output_file, std::string output_path) {
  // clang-format off
  output_file.open(output_path);
  output_file << "x" << ",";
  output_file << "z" << ",";
  output_file << "vx" << ",";
  output_file << "vz" << ",";
  output_file << "theta" << "\n";
  // clang-format on
  return 0;
}

static void record_path(std::ofstream &output_file, VecX path) {
  output_file << path(0) << ",";
  output_file << path(1) << ",";
  output_file << path(2) << ",";
  output_file << path(3) << ",";
  output_file << path(4) << ",";
  output_file << path(5) << "\n";
}

TEST(Trajectory, quadrotor_2d_model) {
  // setup
  std::ofstream output_file;
  trajectory_file_init(output_file, TEST_TRAJECTORY_OUTPUT_FILE);

  // initial state
  VecX x(5);
  x << 0.0, 0.0, 0.0, 0.0, 0.0;

  // initial inputs
  Vec2 u(9.85, 0.0);
  for (int i = 1; i < 10; i++) {
    x = quadrotor_2d_model(x, u);
    record_time_step(output_file, i, x);
  }

  // clean up
  output_file.close();
}

TEST(Trajectory, load_matrix) {
  MatX y;
  std::vector<double> x;

  // setup
  for (int i = 0; i < 10; i++) {
    x.push_back(i);
  }

  // test and assert
  load_matrix(x, 2, 5, y);
  EXPECT_FLOAT_EQ(0.0, y(0, 0));
  EXPECT_FLOAT_EQ(9.0, y(1, 4));
}

TEST(Trajectory, trajectory_calculate_desired_states) {
  struct problem_data p;
  std::ofstream path_file;
  VecX state(5);
  std::vector<double> cost_weights;

  // setup
  path_file_init(path_file, TEST_PATH_OUTPUT_FILE);
  trajectory_setup(&p, 4, 2, 100, cost_weights);
  p.pos_init << -1, 3.5;
  p.pos_final << 2.3, 0;
  p.vel_init << 1.0, 1.0;
  p.vel_final << 0.0, 0.0;
  p.thrust_init = 9.81;
  p.thrust_final = 0.0;
  p.theta_init = 0.0;
  p.theta_final = 0.0;

  // test
  trajectory_calculate_desired(&p);
  for (int i = 0; i < p.nb_steps; i++) {
    state = p.desired.block(0, i, 6, 1);
    record_path(path_file, state);
  }

  // assert
  EXPECT_EQ(p.nb_steps, p.desired.cols());

  state = p.desired.block(0, 0, 6, 1);
  EXPECT_FLOAT_EQ(p.pos_init(0), state(0));
  EXPECT_FLOAT_EQ(p.vel_init(0), state(1));
  EXPECT_FLOAT_EQ(p.pos_init(1), state(2));
  EXPECT_FLOAT_EQ(p.vel_init(1), state(3));
  EXPECT_FLOAT_EQ(p.thrust_init, state(4));
  EXPECT_FLOAT_EQ(p.theta_init, state(5));

  state = p.desired.block(0, p.nb_steps - 1, 6, 1);
  EXPECT_FLOAT_EQ(p.pos_final(0), state(0));
  EXPECT_FLOAT_EQ(p.vel_final(0), state(1));
  EXPECT_FLOAT_EQ(p.pos_final(1), state(2));
  EXPECT_FLOAT_EQ(p.vel_final(1), state(3));
  EXPECT_FLOAT_EQ(p.thrust_final, state(4));
  EXPECT_FLOAT_EQ(p.theta_final, state(5));

  // clean up
  path_file.close();
}

// TEST(Trajectory, trajectory_cost_func) {
//   struct problem_data p;
//   std::vector<double> x;
//   std::vector<double> grad;
//   std::vector<double> cost_weights;
//   double cost;
//
//   // setup
//   cost_weights.push_back(1.0);
//   cost_weights.push_back(1.0);
//   cost_weights.push_back(1.0);
//   trajectory_setup(&p, 4, 2, 10, cost_weights);
//   p.pos_init << -1, 3.5;
//   p.pos_final << 2.3, 0;
//   p.vel_init << 1.0, 1.0;
//   p.vel_final << 0.0, 0.0;
//   p.thrust_init = 9.81;
//   p.thrust_final = 9.81;
//   p.theta_init = 0.2;
//   p.theta_final = 0.0;
//
//   trajectory_calculate_desired(&p);
//   for (int i = 0; i < 10; i++) {
//     x.push_back(p.desired(0, i));  // state - x
//     x.push_back(p.desired(1, i));  // state - vx
//     x.push_back(p.desired(2, i));  // state - z
//     x.push_back(p.desired(3, i));  // state - vz
//     x.push_back(p.desired(4, i));  // input - az
//     x.push_back(p.desired(5, i));  // input - w
//   }
//
//   // test
//   cost = trajectory_cost_func(x, grad, &p);
//   EXPECT_FLOAT_EQ(0.0, cost);
// }
//
// TEST(Trajectory, trajectory_constraint_func) {
//   struct problem_data p;
//   nlopt::opt opt;
//   std::vector<double> x, grad, cost_weights;
//   double error;
//
//   // setup problem
//   trajectory_setup(&p, 4, 2, 10, cost_weights);
//   p.pos_init << -1, 3.5;
//   p.pos_final << 2.3, 0;
//   p.vel_init << 1.0, 1.0;
//   p.vel_final << 0.0, 0.0;
//   p.thrust_init = 9.81;
//   p.thrust_final = 9.81;
//   p.theta_init = 0.2;
//   p.theta_final = 0.0;
//   trajectory_calculate_desired(&p);
//   load_matrix(p.desired, x);
//
//   error = trajectory_constraint_func(x, grad, &p);
//   std::cout << error << std::endl;
// }
//
// TEST(Trajectory, optimize) {
//   double minf;
//   struct problem_data p;
//   nlopt::opt opt;
//   std::vector<double> x;
//   std::vector<double> cost_weights;
//   std::ofstream output_file;
//
//   // setup cost weights
//   cost_weights.push_back(1.0);
//   cost_weights.push_back(1.0);
//   cost_weights.push_back(0.5);
//   cost_weights.push_back(1.0);
//
//   // setup problem
//   trajectory_setup(&p, 4, 2, 100, cost_weights);
//   p.pos_init << -1, 3.5;
//   p.pos_final << 2.3, 0;
//   p.vel_init << 1.0, 1.0;
//   p.vel_final << 0.0, 0.0;
//   p.thrust_init = 9.81;
//   p.thrust_final = 9.81;
//   p.theta_init = 0.2;
//   p.theta_final = 0.0;
//   trajectory_calculate_desired(&p);
//   load_matrix(p.desired, x);
//
//   // configure optimizer
//   opt = nlopt::opt(nlopt::LN_COBYLA, 600);
//   opt.set_min_objective(trajectory_cost_func, &p);
//   opt.add_inequality_constraint(trajectory_constraint_func, &p, 1e-8);
//   opt.set_xtol_rel(1e-4);
//
//   // optimize
//   nlopt::result result = opt.optimize(x, minf);
//
//   // record results
//   trajectory_record_optimization(TEST_OPTIMIZED_OUTPUT_FILE, x, 100);
// }

} // namespace atl
