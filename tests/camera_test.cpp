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
        "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_640.yaml"
    );
    cam.loadConfig(
        "320",
        "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_320.yaml"
    );
    cam.loadConfig(
        "160",
        "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_160.yaml"
    );
    // cam.initCamera("default");
    // cam.initCamera("320");
    cam.initCamera("160");
    cam.run();

    return 0;
}

void test_suite(void)
{
    mu_add_test(test_run);
}

mu_run_tests(test_suite)
