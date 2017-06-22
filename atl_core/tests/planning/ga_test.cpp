#include "atl_core/atl_test.hpp"
#include "atl_core/planning/ga.hpp"

namespace atl {

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

  ASSERT_EQ(2, bit_string.chromosome.size());
  ASSERT_EQ(nb_time_steps, bit_string.nb_time_steps);
}

TEST(GAPopulation, constructor) {
  GAPopulation population;

  ASSERT_EQ(0, population.individuals.size());
}

TEST(GAPopulation, configure) {
  GAPopulation population;

  population.configure(1, 1);
  population.print();

  ASSERT_EQ(1, population.individuals.size());
  ASSERT_EQ(2, population.individuals[0].chromosome.size());
}

TEST(GAProblem, configure) {
  GAProblem problem;
  Vec4 x_init;
  Vec4 x_final;

  // setup
  x_init << 0.0, 0.0, 5.0, 0.0;
  x_final << 5.0, 0.0, 0.0, 0.0;

  // test and assert
  problem.configure(1, 2, 1.0, 1.0, x_init, x_final);

  ASSERT_EQ(1, problem.max_generations);
  ASSERT_EQ(2, problem.tournament_size);
  ASSERT_FLOAT_EQ(1.0, problem.crossover_probability);
  ASSERT_FLOAT_EQ(1.0, problem.mutation_probability);
}

TEST(GAProblem, evaluateIndividual) {
  std::vector<double> cost_weights;
  GAProblem problem;
  GABitString bs;
  Vec4 x_init, x_final;

  // setup
  x_init << 0.0, 0.0, 5.0, 0.0;
  x_final << 5.0, 0.0, 0.0, 0.0;

  // cost weights
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  bs.configure(100);
  problem.configure(10, 2, 1.0, 1.0, x_init, x_final);

  // evaluate
  problem.evaluateIndividual(bs);
  bs.record("/tmp/best.output");
}

TEST(GAProblem, evaluatePopulation) {
  std::vector<double> cost_weights;
  GAProblem problem;
  GAPopulation population;
  GABitString bs;
  Vec4 x_init, x_final;

  // cost weights
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);

  // population
  population.configure(5, 30);

  // evaluate
  problem.configure(10, 2, 1.0, 1.0, x_init, x_final);
  problem.evaluatePopulation(population);
}

TEST(GAProblem, pointCrossover) {
  GAProblem problem;
  GABitString b1;
  GABitString b2;
  Vec4 x_init, x_final;

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

  problem.configure(1, 2, 1.0, 1.0, x_init, x_final);
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
  Vec4 x_init, x_final;

  // setup
  bs.nb_inputs = 2;
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);
  bs.chromosome.push_back(0.0);

  problem.configure(1, 2, 1.0, 1.0, x_init, x_final);
  problem.pointMutation(bs);

  ASSERT_TRUE(bs.chromosome[0] != 0.0);
  ASSERT_TRUE(bs.chromosome[1] != 0.0);
  ASSERT_TRUE(bs.chromosome[2] != 0.0);
  ASSERT_TRUE(bs.chromosome[3] != 0.0);
  ASSERT_TRUE(bs.chromosome[4] != 0.0);
  ASSERT_TRUE(bs.chromosome[5] != 0.0);
}

TEST(GAProblem, tournamentSelection) {
  GAProblem problem;
  GAPopulation population;
  Vec4 x_init, x_final;

  // setup
  population.configure(5, 1);
  population.print();
  std::cout << std::endl;

  problem.configure(1, 2, 1.0, 1.0, x_init, x_final);
  problem.tournamentSelection(population);

  population.print();
}

TEST(GAProblem, findBest) {
  GAProblem problem;
  GAPopulation population;
  GABitString bs;

  // setup
  bs.score = 1.0;
  population.individuals.push_back(bs);
  bs.score = 2.0;
  population.individuals.push_back(bs);
  bs.score = 3.0;
  population.individuals.push_back(bs);
  bs.score = 4.0;
  population.individuals.push_back(bs);
  bs.score = 3.0;
  population.individuals.push_back(bs);
  bs.score = 2.0;
  population.individuals.push_back(bs);
  bs.score = 1.0;
  population.individuals.push_back(bs);

  // test and assert
  problem.findBest(population, bs);
  ASSERT_FLOAT_EQ(4.0, bs.score);
}

TEST(GAProblem, optimize) {
  std::vector<double> cost_weights;
  GAProblem problem;
  GAPopulation population;
  GABitString best;
  Vec4 x_init, x_final;

  // setup
  x_init << 0.0, 0.0, 5.0, 0.0;
  x_final << 5.0, 0.0, 0.0, 0.0;

  // cost weights
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);
  cost_weights.push_back(1.0);

  // population
  population.configure(500, 50);

  // optimize
  problem.configure(1, 10, 0.0, 0.02, x_init, x_final);
  problem.optimize(population);
  problem.findBest(population, best);

  best.record("/tmp/best.output");
}

}  // end of atl namespace
