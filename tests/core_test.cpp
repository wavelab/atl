#ifndef MU_PRINT
  #define MU_PRINT 1
#endif


#include "munit.h"
#include "core.hpp"


int test_core(void)
{
    core(0, NULL);

    return 0;
}


void test_suite()
{
    mu_add_test(test_core);
}

mu_run_tests(test_suite)
