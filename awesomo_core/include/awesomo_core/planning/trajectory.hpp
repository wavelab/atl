#ifndef __AWESOMO_PLANNING_TRAJECTORY_HPP__
#define __AWESOMO_PLANNING_TRAJECTORY_HPP__

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

typedef struct {
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

} trajectory_constraints;

typedef struct {
  int nb_states;
  int nb_time_steps;
  double time_taken;

  Vec3 state_initial;
  Vec3 state_final;
  std::vector<Vec2> desired_trajectory;

} trajectory_data;

Vec2 quadrotor_calculate_inputs(double mass, double thrust, double omega);
VecX quadrotor_2d_model(VecX x, Vec2 u);
int trajectory_problem_dimensions(int nb_states, int nb_timesteps);
int trajectory_sampling_time(double time_taken, int nb_steps);
std::vector<Vec2> trajectory_desired_path(Vec2 pos_init,
                                          Vec2 pos_final,
                                          int nb_steps);

}  // end of awesomo namespace
#endif
