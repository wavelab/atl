#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/ga.hpp"

namespace awesomo {

TEST(problem, setup) {
  struct problem_data p;
  std::vector<double> cost_weights;

  problem_setup(&p, 1, 2, 10, cost_weights);

  ASSERT_EQ(1, p.nb_states);
  ASSERT_EQ(2, p.nb_inputs);
  ASSERT_EQ(10, p.nb_steps);
}

TEST(GABitString, constructor) {
  GABitString bit_string;

  ASSERT_EQ(0, bit_string.chromosome.size());
  ASSERT_EQ(0, bit_string.nb_time_steps);
  ASSERT_FLOAT_EQ(0.0, bit_string.score);
}

TEST(GABitString, configure) {
  int nb_time_steps;
  GABitString bit_string;

  nb_time_steps = 1;
  bit_string.configure(nb_time_steps);
  bit_string.print();

  ASSERT_EQ(6, bit_string.chromosome.size());
  ASSERT_EQ(nb_time_steps, bit_string.nb_time_steps);
}

// TEST(GAPopulation, configure) {
//   GAPopulation population;
//
//   population.configure(1, 1);
//   population.print();
//
//   ASSERT_EQ(1, population.individuals.size());
//   ASSERT_EQ(6, population.individuals[0].chromosome.size());
// }

TEST(GAProblem, configure) {
  GAProblem problem;

  // test and assert
  problem.configure(1, 2, 1.0, 1.0);
  ASSERT_EQ(1, problem.max_generations);
  ASSERT_EQ(2, problem.tournament_size);
  ASSERT_FLOAT_EQ(1.0, problem.crossover_probability);
  ASSERT_FLOAT_EQ(1.0, problem.mutation_probability);
}

TEST(GAProblem, calculateDesired) {
  struct problem_data p;
  std::vector<double> cost_weights;
  GAProblem problem;

  problem_setup(&p, 4, 2, 1, cost_weights);
  p.pos_init << -1, 3.5;
  p.pos_final << 2.3, 0;
  p.vel_init << 1.0, 1.0;
  p.vel_final << 0.0, 0.0;
  p.thrust_init = 9.81;
  p.thrust_final = 0.0;
  p.theta_init = 0.0;
  p.theta_final = 0.0;

  problem.calculateDesired(&p);
}

TEST(GAProblem, evaluateIndividual) {
  struct problem_data p;
  std::vector<double> cost_weights;
  GAProblem problem;
  GABitString bs;

  // cost weights
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);

  // problem data
  problem_setup(&p, 4, 2, 5, cost_weights);
  p.pos_init << -1, 3.5;
  p.pos_final << 2.3, 0;
  p.vel_init << 1.0, 1.0;
  p.vel_final << 0.0, 0.0;
  p.thrust_init = 9.81;
  p.thrust_final = 9.81;
  p.theta_init = 0.0;
  p.theta_final = 0.0;
  problem.calculateDesired(&p);
  load_matrix(p.desired, bs.chromosome);

  // evaluate
  problem.evaluateIndividual(bs, &p);
  ASSERT_FLOAT_EQ(0.0, bs.score);
}

TEST(GAProblem, evaluatePopulation) {
  struct problem_data data;
  std::vector<double> cost_weights;
  GAProblem problem;
  GAPopulation population;
  GABitString bs;

  // cost weights
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);

  // problem data
  problem_setup(&data, 4, 2, 5, cost_weights);
  data.pos_init << -1, 3.5;
  data.pos_final << 2.3, 0;
  data.vel_init << 1.0, 1.0;
  data.vel_final << 0.0, 0.0;
  data.thrust_init = 9.81;
  data.thrust_final = 9.81;
  data.theta_init = 0.0;
  data.theta_final = 0.0;
  problem.calculateDesired(&data);

  // population
  population.configure(5, &data);

  // evaluate
  problem.evaluatePopulation(population, &data);
}

TEST(GAProblem, pointCrossover) {
  GAProblem problem;
  GABitString b1;
  GABitString b2;

  // setup
  b1.chromosome.push_back(1.0);
  b1.chromosome.push_back(1.0);
  b1.chromosome.push_back(1.0);
  b1.chromosome.push_back(1.0);
  b1.chromosome.push_back(1.0);
  b1.chromosome.push_back(1.0);
  // b1.print();
  // std::cout << std::endl;

  b2.chromosome.push_back(0.0);
  b2.chromosome.push_back(0.0);
  b2.chromosome.push_back(0.0);
  b2.chromosome.push_back(0.0);
  b2.chromosome.push_back(0.0);
  b2.chromosome.push_back(0.0);
  // b2.print();
  // std::cout << std::endl;

  problem.configure(1, 2, 1.0, 1.0);
  problem.pointCrossover(b1, b2);

  ASSERT_FLOAT_EQ(0.0, b1.chromosome[0]);
  ASSERT_FLOAT_EQ(0.0, b1.chromosome[1]);
  ASSERT_FLOAT_EQ(0.0, b1.chromosome[2]);
  ASSERT_FLOAT_EQ(1.0, b1.chromosome[3]);
  ASSERT_FLOAT_EQ(1.0, b1.chromosome[4]);
  ASSERT_FLOAT_EQ(1.0, b1.chromosome[5]);

  ASSERT_FLOAT_EQ(1.0, b2.chromosome[0]);
  ASSERT_FLOAT_EQ(1.0, b2.chromosome[1]);
  ASSERT_FLOAT_EQ(1.0, b2.chromosome[2]);
  ASSERT_FLOAT_EQ(0.0, b2.chromosome[3]);
  ASSERT_FLOAT_EQ(0.0, b2.chromosome[4]);
  ASSERT_FLOAT_EQ(0.0, b2.chromosome[5]);
}

TEST(GAProblem, pointMutation) {
  GAProblem problem;
  GABitString bs;

  // setup
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);

  problem.configure(1, 2, 1.0, 1.0);
  problem.pointMutation(bs);

  ASSERT_TRUE(bs.chromosome[0] != 0.0);
  ASSERT_TRUE(bs.chromosome[1] != 0.0);
  ASSERT_TRUE(bs.chromosome[2] != 0.0);
  ASSERT_TRUE(bs.chromosome[3] != 0.0);
  ASSERT_TRUE(bs.chromosome[4] != 0.0);
  ASSERT_TRUE(bs.chromosome[5] != 0.0);
}

// TEST(GAProblem, tournamentSelection) {
//   GAProblem problem;
//   GAPopulation population;
//
//   // setup
//   population.configure(5, 1);
//   population.print();
//   std::cout << std::endl;
//
//   problem.configure(1, 2, 1.0, 1.0);
//   problem.tournamentSelection(population);
//
//   // population.print();
// }

// TEST(GAProblem, findBest) {
//   GAProblem problem;
//   GAPopulation population;
//   GABitString bs;
//
//   // setup
//   bs.score = 1.0;
//   population.individuals.push_back(bs);
//   bs.score = 2.0;
//   population.individuals.push_back(bs);
//   bs.score = 3.0;
//   population.individuals.push_back(bs);
//   bs.score = 4.0;
//   population.individuals.push_back(bs);
//   bs.score = 3.0;
//   population.individuals.push_back(bs);
//   bs.score = 2.0;
//   population.individuals.push_back(bs);
//   bs.score = 1.0;
//   population.individuals.push_back(bs);
//
//   // test and assert
//   problem.findBest(population, bs);
//   ASSERT_FLOAT_EQ(4.0, bs.score);
// }

TEST(GAProblem, optimize) {
  struct problem_data data;
  std::vector<double> cost_weights;
  GAProblem problem;
  GAPopulation population;
  GABitString best;

  // cost weights
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);

  // problem data
  problem_setup(&data, 4, 2, 50, cost_weights);
  data.pos_init << -1, 3.5;
  data.pos_final << 2.3, 0;
  data.vel_init << 1.0, 1.0;
  data.vel_final << 0.0, 0.0;
  data.thrust_init = 9.81;
  data.thrust_final = 0.0;
  data.theta_init = 0.0;
  data.theta_final = 0.0;
  problem.calculateDesired(&data);

  // population
  population.configure(200, &data);

  // optimize
  problem.configure(500, 10, 0.0, 0.01);
  problem.optimize(population, &data);
  problem.findBest(population, best);

  best.record("/tmp/best.output");
}

}  // end of awesomo namespace
