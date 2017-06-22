#ifndef __atl_PLANNING_GA_HPP__
#define __atl_PLANNING_GA_HPP__

#include <iostream>
#include <fstream>
#include <vector>

#include "atl_core/utils/utils.hpp"
#include "atl_core/planning/model.hpp"


namespace atl {

class GABitString {
public:
  std::vector<double> chromosome;
  int nb_inputs;
  int nb_time_steps;

  MatX X;
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
  int configure(int nb_individuals, int nb_time_steps);
  void print(void);
};

class GAProblem {
public:
  bool configured;

  int max_generations;
  int tournament_size;
  double crossover_probability;
  double mutation_probability;

  Simulator sim;
  Vec4 x_init;
  Vec4 x_final;

  GAProblem(void);
  int configure(int max_generations,
                int tournament_size,
                double crossover_probability,
                double mutation_probability,
                Vec4 x_init,
                Vec4 x_final);
  int evaluateIndividual(GABitString &bs);
  int evaluatePopulation(GAPopulation &p);
  int tournamentSelection(GAPopulation &p);
  int pointCrossover(GABitString &b1, GABitString &b2);
  int pointMutation(GABitString &bs);
  void findBest(GAPopulation &p, GABitString &best);
  int optimize(GAPopulation &p);
};

}  // end of atl namespace
#endif
