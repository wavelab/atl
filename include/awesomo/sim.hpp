#ifndef __SIM_HPP__
#define __SIM_HPP__

#include <iostream>

#include "awesomo/util.hpp"
#include "awesomo/quadrotor.hpp"


// CONSTANTS
#ifndef MATH_PI
  #define MATH_PI 3.14159265358979323846
#endif


// STRUCTURES
struct world
{
    double dt;
    double gravity;
};


// FUNCTIONS
void sim_record_step(FILE *out_file, struct quadrotor *q);
void sim_loop(void);

#endif
