#ifndef __AWESOMO_PLANNING_OPTIMIZER_HPP__
#define __AWESOMO_PLANNING_OPTIMIZER_HPP__

#include <math.h>

#include <iostream>

#include <nlopt.hpp>


namespace awesomo {

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

}  // end of awesomo namespace
#endif
