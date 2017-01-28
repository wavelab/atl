#ifndef __AWESOMO_PLANNING_GA_HPP__
#define __AWESOMO_PLANNING_GA_HPP__

#include <iostream>
#include <fstream>
#include <vector>

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

struct problem_data {
  int nb_states;
  int nb_inputs;
  int nb_steps;

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

void problem_setup(struct problem_data *p,
                   int nb_states,
                   int nb_inputs,
                   int nb_steps,
                   std::vector<double> cost_weights);

class GABitString {
public:
  std::vector<double> chromosome;
  int nb_time_steps;
  double score;

  GABitString(void);
  int configure(int nb_time_steps);
  int record(std::string file_path);
  void print(void);
};

class GAPopulation {
public:
  std::vector<GABitString> individuals;

  GAPopulation(void);
  int configure(int nb_individuals, struct problem_data *data);
  void print(void);
};

class GAProblem {
public:
  bool configured;

  int max_generations;
  int tournament_size;
  double crossover_probability;
  double mutation_probability;

  GAProblem(void);
  int configure(int max_generations,
                int tournament_size,
                double crossover_probability,
                double mutation_probability);
  int calculateDesired(struct problem_data *p);
  int evaluateIndividual(GABitString &bs, struct problem_data *data);
  int evaluatePopulation(GAPopulation &p, struct problem_data *data);
  int tournamentSelection(GAPopulation &p);
  int pointCrossover(GABitString &b1, GABitString &b2);
  int pointMutation(GABitString &bs);
  void findBest(GAPopulation &p, GABitString &best);
  int optimize(GAPopulation &p, struct problem_data *data);
};

}  // end of awesomo namespace
#endif
