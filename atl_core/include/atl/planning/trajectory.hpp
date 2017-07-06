#ifndef ATL_PLANNING_TRAJECTORY_HPP
#define ATL_PLANNING_TRAJECTORY_HPP

#include <iostream>
#include <fstream>

#include "atl/utils/utils.hpp"

namespace atl {

struct problem_constraints {
  // input constraints
  double thrust_min;
  double thrust_max;

  double angular_rate_min;
  double angular_rate_max;

  // feasible region constraints
  double feasible_thrust_min;
  double feasible_thrust_max;

  double feasible_angular_rate_min;
  double feasible_angular_rate_max;
};

struct problem_data {
  int nb_states;
  int nb_inputs;
  int nb_steps;
  double time_taken;

  Vec2 pos_init;
  Vec2 pos_final;
  Vec2 vel_init;
  Vec2 vel_final;
  double thrust_init;
  double thrust_final;
  double theta_init;
  double theta_final;

  MatX desired;
  std::vector<double> cost_weights;
};

Vec2 quadrotor_calculate_inputs(double mass, double thrust, double omega);
VecX quadrotor_2d_model(VecX x, Vec2 u);

void trajectory_setup(struct problem_data *p,
                      int nb_states,
                      int nb_inputs,
                      int nb_steps,
                      std::vector<double> cost_weights);
int trajectory_calculate_desired(struct problem_data *p);
double trajectory_cost_func(const std::vector<double> &x,
                            std::vector<double> &grad,
                            void *data);
double trajectory_constraint_func(const std::vector<double> &x,
                                  std::vector<double> &grad,
                                  void *data);
int trajectory_record_optimization(std::string file_path,
                                   std::vector<double> x,
                                   int nb_rows);


}  // namespace atl
#endif
