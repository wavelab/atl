#include "munit.h"
#include "imu.hpp"

// TESTS
int testImu(void);

int testImu(void)
{
    // if (check_apm()) {
    //     return 1;
    // }
    //
	// MPU9250 imu;
	// imu.initialize();
    //
	// float ax, ay, az;
	// float gx, gy, gz;
	// float mx, my, mz;
    //
    // while (1) {
    //     imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	// 	printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
	// 	printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
	// 	printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
    //
	// 	usleep(500000);
    // }

	return 0;
}

void testSuite(void)
{
    mu_add_test(testImu);
}

mu_run_tests(testSuite)
