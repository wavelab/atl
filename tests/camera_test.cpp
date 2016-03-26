#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
int test_run(void);
void test_suite(void);


int test_run(void)
{
    Camera cam(
        0,
        CAMERA_FIREFLY,
        // "/home/stan/Projects/awesomo/configs/firefly_calib/ost.yaml"
        "/home/chutsu/Dropbox/proj/awesomo/configs/firefly_calib/ost.yaml"
    );
    // Camera cam(
    //     0,
    //     CAMERA_NORMAL,
    //     "/home/chutsu/Dropbox/proj/awesomo/configs/ost.yml"
    // );
    cam.run();

    return 0;
}

void test_suite(void)
{
    mu_add_test(test_run);
}

mu_run_tests(test_suite)
