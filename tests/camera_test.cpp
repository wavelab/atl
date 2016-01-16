#ifndef MU_PRINT
  #define MU_PRINT 1
#endif


#include "munit.h"
#include "camera.hpp"


int test_camera_run(void)
{
    Camera cam;

    cam.run();

    return 0;
}


void test_suite()
{
    mu_add_test(test_camera_run);
}

mu_run_tests(test_suite)
