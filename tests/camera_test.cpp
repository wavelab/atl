#include "awesomo/munit.h"
#include "awesomo/camera.hpp"

#define CONFIG_PATH "configs/pointgrey_firefly"


// TESTS
int testRun(void);
int testCameraloadConfig(void);
void testSuite(void);


int testCameraloadConfig(void)
{
    Camera cam(CONFIG_PATH);

    return 0;
}

int testRun(void)
{
    Camera cam(CONFIG_PATH);
    cam.run();

    return 0;
}

void testSuite(void)
{
    mu_add_test(testCameraloadConfig);
    mu_add_test(testRun);
}

mu_run_tests(testSuite)
