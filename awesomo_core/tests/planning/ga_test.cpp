#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/ga.hpp"

namespace awesomo {

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

TEST(GAPopulation, configure) {
  GAPopulation population;

  population.configure(1, 1);
  population.print();

  ASSERT_EQ(1, population.individuals.size());
  ASSERT_EQ(6, population.individuals[0].chromosome.size());
}

TEST(GAProblem, configure) {
  GAProblem problem;

  // test and assert
  problem.configure(1, 2, 1.0, 1.0);
  ASSERT_EQ(1, problem.max_generation);
  ASSERT_EQ(2, problem.tournament_size);
  ASSERT_FLOAT_EQ(1.0, problem.crossover_probability);
  ASSERT_FLOAT_EQ(1.0, problem.mutation_probability);
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

TEST(GAProblem, tournamentSelection) {
  GAProblem problem;
  GAPopulation population;

  // setup
  population.configure(5, 1);
  population.print();
  std::cout << std::endl;

  problem.configure(1, 2, 1.0, 1.0);
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

}  // end of awesomo namespace
