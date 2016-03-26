#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
int test_run(void);
void test_suite(void);


int test_run(void)
{
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig(
        "default",
        "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost.yaml"
    );
    cam.initCamera("default");
    cam.run();

    return 0;
}

void test_suite(void)
{
    mu_add_test(test_run);
}

mu_run_tests(test_suite)
