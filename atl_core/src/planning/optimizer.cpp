#include "atl/planning/optimizer.hpp"

namespace atl {

double cost_func(
  const std::vector<double> &x, std::vector<double> &grad, void *data) {
  UNUSED(data);

  // calculate gradients
  if (!grad.empty()) {
    grad[0] = 0.0;
    grad[1] = 0.5 / sqrt(x[1]);
  }

  return sqrt(x[1]);
}

double constraint_func(
  const std::vector<double> &x, std::vector<double> &grad, void *data) {
  constraint_data *d;
  double a, b;

  // setup
  d = reinterpret_cast<constraint_data *>(data);
  a = d->a;
  b = d->b;

  // calculate gradients
  if (!grad.empty()) {
    grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
    grad[1] = -1.0;
  }

  return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
}

POpt::POpt() {}

int POpt::run() {
  double minf;
  std::vector<double> lb(2);
  std::vector<double> x(2);

  // setup
  lb[0] = -HUGE_VAL;
  lb[1] = 0;
  x[0] = 1.234;
  x[1] = 5.678;
  constraint_data data[2] = {{2, 0}, {-1, 1}};

  // configure optimizer
  this->opt = nlopt::opt(nlopt::LD_MMA, 2);
  this->opt.set_lower_bounds(lb);
  this->opt.set_min_objective(cost_func, NULL);

  this->opt.add_inequality_constraint(constraint_func, &data[0], 1e-8);
  this->opt.add_inequality_constraint(constraint_func, &data[1], 1e-8);
  this->opt.set_xtol_rel(1e-4);

  // optimize
  nlopt::result result = this->opt.optimize(x, minf);
  if (result < 0) {
    return -1;
  }

  std::cout << "x[0]: " << x[0] << std::endl;
  std::cout << "x[1]: " << x[1] << std::endl;
  std::cout << "result: " << minf << std::endl;

  return 0;
}

}  // namespace atl
