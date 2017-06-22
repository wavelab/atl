#include "atl/planning/trajectory.hpp"


namespace atl {

Vec2 quadrotor_calculate_inputs(double mass, double thrust, double omega) {
  Vec2 u;

  u(0) = thrust / mass;
  u(1) = omega;

  return u;
}

VecX quadrotor_2d_model(VecX x, Vec2 u) {
  const double k_gravity = 9.81;

  // x1 - x
  // x2 - z
  // x3 - vx
  // x4 - vz
  // x5 - theta (pitch)

  x(0) = x(0) + x(1);
  x(1) = x(1) + u(0) * sin(x(4));
  x(2) = x(2) + x(3);
  x(3) = x(3) + u(0) * cos(x(4)) - k_gravity;
  x(4) = x(4) + u(1);

  return x;
}

void trajectory_setup(struct problem_data *p,
                      int nb_states,
                      int nb_inputs,
                      int nb_steps,
                      std::vector<double> cost_weights) {
  p->nb_states = nb_states;
  p->nb_inputs = nb_inputs;

  p->nb_steps = nb_steps;
  p->time_taken = 0.0;

  p->pos_init << 0.0, 0.0;
  p->pos_final << 0.0, 0.0;
  p->vel_init << 0.0, 0.0;
  p->vel_final << 0.0, 0.0;
  p->theta_init = 0.0;
  p->theta_final = 0.0;

  p->desired.resize(nb_states + nb_inputs, nb_steps);
  p->cost_weights = cost_weights;
}

int trajectory_calculate_desired(struct problem_data *p) {
  double dx;
  VecX x(6);
  double m, c;

  // calculate line equation, gradient and intersect
  m = (p->pos_init(1) - p->pos_final(1)) / (p->pos_init(0) - p->pos_final(0));
  c = p->pos_init(1) - m * p->pos_init(0);

  // push initial x
  x(0) = p->pos_init(0);  // state - x
  x(1) = p->vel_init(0);  // state - vx
  x(2) = p->pos_init(1);  // state - z
  x(3) = p->vel_init(1);  // state - vz
  x(4) = p->thrust_init;  // input - az
  x(5) = p->theta_init;   // input - w
  p->desired.block(0, 0, 6, 1) = x;

  // create points along the desired line path
  dx = (p->pos_final(0) - p->pos_init(0)) / (double) (p->nb_steps - 1);
  for (int i = 0; i < (p->nb_steps - 2); i++) {
    x = p->desired.block(0, i, 6, 1);

    x(0) += dx;             // state - x
    x(1) = p->vel_init(0);  // state - vx
    x(2) = m * x(0) + c;    // state - z
    x(3) = p->vel_init(1);  // state - vz
    x(4) = p->thrust_init;  // input - az
    x(5) = p->theta_init;   // input - w

    p->desired.block(0, i + 1, 6, 1) = x;
  }

  // push final x
  x(0) = p->pos_final(0);   // state - x
  x(1) = p->vel_final(0);   // state - vx
  x(2) = p->pos_final(1);   // state - z
  x(3) = p->vel_final(1);   // state - vz
  x(4) = p->thrust_final;   // input - az
  x(5) = p->theta_final;    // input - w
  p->desired.block(0, p->nb_steps - 1, 6, 1) = x;

  return 0;
}

double trajectory_cost_func(const std::vector<double> &x,
                            std::vector<double> &grad,
                            void *data) {
  double cost;
  struct problem_data *p;
  MatX X;
  VecX g;
  VecX x_opt, x_des;
  VecX z_opt, z_des;
  VecX u1_opt, u2_opt;

  // setup
  cost = 0.0;
  p = (struct problem_data *) data;
  load_matrix(x, p->nb_states + p->nb_inputs, p->nb_steps, X);
  g = 9.81 * MatX::Ones(p->nb_steps, 1);

  // position error cost
  x_opt = X.row(0);
  x_des = p->desired.row(0);
  cost += p->cost_weights[0] * (x_opt - x_des).norm();

  z_opt = X.row(2);
  z_des = p->desired.row(2);
  cost += p->cost_weights[1] * (z_opt - z_des).norm();

  // control input cost
  u1_opt = X.row(4);
  u2_opt = X.row(5);
  cost += p->cost_weights[2] * (u1_opt - g).squaredNorm();
  cost += p->cost_weights[3] * u2_opt.squaredNorm();

  return cost;
}

double trajectory_constraint_func(const std::vector<double> &x,
                                  std::vector<double> &grad,
                                  void *data) {
  struct problem_data *p;
  double error, dt;
  MatX X;
  Vec4 x_curr, x_prev, x_dot;
  Vec2 u_prev;

  // setup
  error = 0.0;
  dt = 0.1;
  p = (struct problem_data *) data;
  load_matrix(x, p->nb_states + p->nb_inputs, p->nb_steps, X);

  for (int i = 0; i < (p->nb_steps - 1); i++) {
    u_prev = X.block(4, i, 2, 1);
    x_prev = X.block(0, i, 4, 1);
    x_curr = X.block(0, i + 1, 4, 1);

    // clang-format off
    x_dot << x_prev(1),
             u_prev(0) * sin(u_prev(1)),
             x_prev(3),
             u_prev(0) * cos(u_prev(1));
    // clang-format on

    // calculate feasible regions
    error += ((x_curr - x_prev) - dt * x_dot).sum();
  }

  return error;
}

int trajectory_record_optimization(std::string file_path,
                                   std::vector<double> x,
                                   int nb_rows) {
  std::ofstream output_file;
  int nb_states;

  // setup
  output_file.open(file_path);
  output_file << "time_step" << ",";
  output_file << "x" << ",";
  output_file << "vx" << ",";
  output_file << "z" << ",";
  output_file << "vz" << ",";
  output_file << "az" << ",";
  output_file << "theta" << "\n";

  // record
  nb_states = 6;
  for (int i = 0; i < nb_rows; i++) {
    output_file << i << ",";
    output_file << x[i * nb_states + 0] << ",";
    output_file << x[i * nb_states + 1] << ",";
    output_file << x[i * nb_states + 2] << ",";
    output_file << x[i * nb_states + 3] << ",";
    output_file << x[i * nb_states + 4] << ",";
    output_file << x[i * nb_states + 5] << "\n";
  }

  // clean up
  output_file.close();

  return 0;
}

}  // end of atl namespace
