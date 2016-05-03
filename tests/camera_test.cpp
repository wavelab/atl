#include "awesomo/munit.h"
#include "awesomo/camera.hpp"


// TESTS
int testRun(void);
int testCameraloadConfig(void);
void testSuite(void);


int testCameraloadConfig(void)
{
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig("default", FIREFLY_640);

    return 0;
}

int testRun(void)
{
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig("default", FIREFLY_640);
    cam.loadConfig("320", FIREFLY_320);
    cam.loadConfig("160", FIREFLY_160);
    cam.initCamera("160");
    cam.run();

    return 0;
}

void testSuite(void)
{
    mu_add_test(testCameraloadConfig);
    mu_add_test(testRun);
}

mu_run_tests(testSuite)
