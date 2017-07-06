#ifndef ATL_PLANNING_OPTIMIZER_HPP
#define ATL_PLANNING_OPTIMIZER_HPP

#include <math.h>

#include <iostream>

#include <nlopt.hpp>

#include "atl/utils/utils.hpp"


namespace atl {

typedef struct {
  double a;
  double b;
} constraint_data;

class POpt {
public:
  nlopt::opt opt;

  POpt(void);
  int run(void);
};

}  // namespace atl
#endif
