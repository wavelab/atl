#ifndef __AWESOMO_PLANNING_GA_HPP__
#define __AWESOMO_PLANNING_GA_HPP__

#include <iostream>
#include <vector>

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class GABitString {
public:
  std::vector<double> chromosome;
  int nb_time_steps;
  double score;

  GABitString(void);
  int configure(int nb_time_steps);
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

  GAProblem(void);
  int configure(int max_generations,
                int tournament_size,
                double crossover_probability,
                double mutation_probability);
  int evaluatePopulation(GAPopulation &p);
  int evaluateIndividual(GABitString &bs);
  int tournamentSelection(GAPopulation &p);
  int pointCrossover(GABitString &b1, GABitString &b2);
  int pointMutation(GABitString &bs);
  void findBest(GAPopulation &p, GABitString &best);
  int optimize(GAPopulation &p);
};

}  // end of awesomo namespace
#endif
