#ifndef __atl_PLANNING_OPTIMIZER_HPP__
#define __atl_PLANNING_OPTIMIZER_HPP__

#include <math.h>

#include <iostream>

#include <nlopt.hpp>


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

}  // end of atl namespace
#endif
