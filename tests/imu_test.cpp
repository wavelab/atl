#include "munit.h"
#include "imu.hpp"

// TESTS
int testImu(void);

int testImu(void)
{
    IMU imu;

    while (1) {
        imu.read();
        imu.print();
		usleep(500000);
    }

	return 0;
}

void testSuite(void)
{
    mu_add_test(testImu);
}

mu_run_tests(testSuite)
