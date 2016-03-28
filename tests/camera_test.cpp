#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
int testRun(void);
int testCameraloadConfig(void);
void testSuite(void);


int testCameraloadConfig(void)
{
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig("test", "tests/config/ost.yaml");


    return 0;
}

int testRun(void)
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

void testSuite(void)
{
    // mu_add_test(testCameraloadConfig);
    mu_add_test(testRun);
}

mu_run_tests(testSuite)
