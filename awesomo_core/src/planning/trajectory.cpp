#include "awesomo_core/planning/trajectory.hpp"


namespace awesomo {

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

int trajectory_problem_dimensions(int nb_states, int nb_timesteps) {
  return nb_states * nb_timesteps;
}

int trajectory_sampling_time(double time_taken, int nb_steps) {
  return (time_taken) / ((double) nb_steps - 1);
}

std::vector<Vec2> trajectory_desired_path(Vec2 pos_init,
                                          Vec2 pos_final,
                                          int nb_steps) {
  double dx;
  Vec2 point;
  double m, c;
  std::vector<Vec2> desired_path;

  // calculate line equation, gradient and intersect
  m = (pos_init(1) - pos_final(1)) / (pos_init(0) - pos_final(0));
  c = pos_init(1) - m * pos_init(0);

  // create points along the desired line path
  dx = (pos_final(0) - pos_init(0)) / (double) (nb_steps - 1);
  desired_path.push_back(pos_init);
  for (int i = 0; i < (nb_steps - 2); i++) {
    point = desired_path.back();
    point(0) += dx;
    point(1) = m * point(0) + c;
    desired_path.push_back(point);
  }
  desired_path.push_back(pos_final);

  return desired_path;
}

double trajectory_cost_func(const std::vector<double> &x,
                            std::vector<double> &grad,
                            void *data) {
  // // position error cost
  // f = f + w_cost(1)*norm(traj_d(:,1) - x(1:N:end))';
  // f = f + w_cost(2)*norm(traj_d(:,3) - x(3:N:end))';
  //
  // // control input cost
  // f = f + w_cost(3)*sum((x(5:N:end)-g).^2);
  // f = f + w_cost(4)*sum(x(6:N:end).^2);
  //
  // // control input change cost
  // f = f + w_cost(5)*sum(diff(x(5:N:end)).^2);
  // f = f + w_cost(6)*sum(diff(x(6:N:end)).^2);
  //
  // // end cost to bias to matched landing
  // f = f + w_cost(7)*(x(end).^2);

  double cost;

  // setup
  cost = 0.0;






  return cost;
}

}  // end of awesomo namespace
