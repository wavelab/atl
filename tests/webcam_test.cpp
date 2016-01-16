#ifndef MU_PRINT
  #define MU_PRINT 1
#endif


#include "munit.h"
#include "webcam.h"


void test_suite()
{
    Camera cam;
    cam.run();
}

mu_run_tests(test_suite)
