#include "atl_core/planning/ga.hpp"


namespace atl {

GABitString::GABitString(void) {
  this->chromosome.clear();
  this->nb_inputs = 0;
  this->nb_time_steps = 0;
  this->score = 0.0;
}

int GABitString::configure(int nb_time_steps) {
  int nb_inputs;
  double rand_az;
  double rand_theta;

  // setup
  this->nb_inputs = 2;
  this->nb_time_steps = nb_time_steps;
  this->X.resize(4, nb_time_steps);
  rand_az = randf(0.0, 5.0);
  rand_theta = randf(deg2rad(-40.0), deg2rad(40.0));

  // initialize chromosome
  for (int i = 0; i < this->nb_time_steps; i++) {
    this->chromosome.push_back(rand_az);     // az
    this->chromosome.push_back(rand_theta);  // theta
  }
}

int GABitString::record(std::string file_path) {
  std::ofstream output_file;
  int m;

  // setup
  output_file.open(file_path);
  output_file << "time_step" << ",";
  output_file << "x" << ",";
  output_file << "vx" << ",";
  output_file << "z" << ",";
  output_file << "vz" << ",";
  output_file << "az" << ",";
  output_file << "theta" << std::endl;

  // record
  for (int i = 0; i < this->nb_time_steps; i++) {
    output_file << i << ",";
    output_file << this->X(0, i) << ",";
    output_file << this->X(1, i) << ",";
    output_file << this->X(2, i) << ",";
    output_file << this->X(3, i) << ",";
    output_file << this->chromosome[i * 2 + 0] << ",";
    output_file << this->chromosome[i * 2 + 1] << std::endl;
  }

  // clean up
  output_file.close();

  return 0;
}

void GABitString::print(void) {
  for (int i = 0; i < this->chromosome.size(); i++) {
    std::cout << "[" << i << "]: " << this->chromosome[i] << std::endl;
  }
}

GAPopulation::GAPopulation(void) {
  this->individuals.clear();
}

int GAPopulation::configure(int nb_individuals, int nb_time_steps) {
  for (int i = 0; i < nb_individuals; i++) {
    GABitString bs;
    bs.configure(nb_time_steps);
    this->individuals.push_back(bs);
  }
}

void GAPopulation::print(void) {
  for (int i = 0; i < this->individuals.size(); i++) {
    std::cout << "individual[" << i << "]: ";
    std::cout << "score: " << this->individuals[i].score << std::endl;
  }
}

GAProblem::GAProblem(void) {
  this->configured = false;

  this->max_generations = 0;
  this->tournament_size = 0;
  this->crossover_probability = 0.0;
  this->mutation_probability = 0.0;

  this->x_init << 0.0, 0.0, 0.0, 0.0;
  this->x_final << 0.0, 0.0, 0.0, 0.0;
}

int GAProblem::configure(int max_generations,
                         int tournament_size,
                         double crossover_probability,
                         double mutation_probability,
                         Vec4 x_init,
                         Vec4 x_final) {

  // seed random
  srand(time(NULL));

  // fields
  this->max_generations = max_generations;
  this->tournament_size = tournament_size;
  this->crossover_probability = crossover_probability;
  this->mutation_probability = mutation_probability;

  this->sim.configure(x_init, x_final, 1.0);
  this->x_init = x_init;
  this->x_final = x_final;

  this->configured = true;
  return 0;
}

int GAProblem::evaluateIndividual(GABitString &bs) {
  double dt, tend;
  MatX U;

  // setup
  dt = 0.1;
  tend = dt * bs.nb_time_steps;
  load_matrix(bs.chromosome, 2, bs.nb_time_steps, U);

  // evaluate
  sim.simulate(dt, tend, U, bs.X);
  bs.score = 0;
  // bs.score -= 10 * sim.d_az;
  // bs.score -= 10 * sim.d_az;
  // bs.score -= 10 * sim.d_theta;
  // bs.score -= 10 * sim.az_sum;
  bs.score -= 1000 * sim.dist_error;
  bs.score -= 1000 * sim.vel_error;

  return 0;
}

int GAProblem::evaluatePopulation(GAPopulation &p) {
  // evaluate individuals
  for (int i = 0; i < p.individuals.size(); i++) {
    this->evaluateIndividual(p.individuals[i]);
  }

  return 0;
}

int GAProblem::pointCrossover(GABitString &b1, GABitString &b2) {
  int xpoint;
  std::vector<double> chromosome_copy;

  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->crossover_probability < randf(0.0, 1.0)) {
    return 0;
  }

  // setup
  xpoint = b1.chromosome.size() / 2;
  chromosome_copy = b1.chromosome;

  // point crossover
  for (int i = 0; i < xpoint; i++) {
    b1.chromosome[i] = b2.chromosome[i];
  }

  for (int i = 0; i < xpoint; i++) {
    b2.chromosome[i] = chromosome_copy[i];
  }

  return 0;
}

int GAProblem::pointMutation(GABitString &bs) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // point mutation
  for (int i = 0; i < (bs.chromosome.size() / bs.nb_inputs); i++) {
    // az
    if (this->mutation_probability > randf(0.0, 1.0)) {
      bs.chromosome[i * bs.nb_inputs + 0] = randf(0.0, 5.0);
    }

    // theta
    if (this->mutation_probability > randf(0.0, 1.0)) {
      bs.chromosome[i * bs.nb_inputs + 1] = randf(deg2rad(-40.0),
                                                  deg2rad(40.0));
    }
  }

  return 0;
}

int GAProblem::tournamentSelection(GAPopulation &p) {
  GABitString bs;
  GABitString best;
  std::vector<GABitString> tournament;
  std::vector<GABitString> selection;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // tournament selection
  for (int i = 0; i < p.individuals.size(); i++) {
    // create tournament
    for (int j = 0; j < this->tournament_size; j++) {
      bs = p.individuals[randi(0, p.individuals.size() - 1)];
      tournament.push_back(bs);
    }

    // find best in tournament
    best = tournament[0];
    for (int j = 0; j < this->tournament_size - 1; j++) {
      if (tournament[j].score > best.score) {
        best = tournament[j];
      }
    }

    // add to selection and clear tournament
    selection.push_back(best);
    tournament.clear();
  }

  // replace population with selected individuals
  p.individuals = selection;

  return 0;
}

void GAProblem::findBest(GAPopulation &p, GABitString &best) {
  // pre-load best
  best = p.individuals[0];

  // find best
  for (int i = 1; i < p.individuals.size(); i++) {
    if (p.individuals[i].score > best.score) {
      best = p.individuals[i];
    }
  }
}

int GAProblem::optimize(GAPopulation &p) {
  GABitString best;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // optimize
  for (int i = 0; i < this->max_generations; i++) {
    this->evaluatePopulation(p);
    this->tournamentSelection(p);

    for (int j = 0; j < (p.individuals.size() / 2); j++) {
      this->pointCrossover(p.individuals[j * 2], p.individuals[j * 2 + 1]);
      this->pointMutation(p.individuals[j * 2]);
      this->pointMutation(p.individuals[j * 2 + 1]);
    }

    this->findBest(p, best);
    std::cout << "gen: " << i << " ";
    std::cout << "score: " << best.score << std::endl;
  }

  return 0;
}

}  // end of atl namespace
