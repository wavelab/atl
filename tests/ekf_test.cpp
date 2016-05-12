#include "munit.h"
#include "ekf.hpp"


// TESTS
int testPredictionUpdate(void);
int testMeasurementUpdate(void);


int testPredictionUpdate(void)
{

	return 0;
}

int testMeasurementUpdate(void)
{

	return 0;
}

void testSuite(void)
{
    mu_add_test(testPredictionUpdate);
    mu_add_test(testMeasurementUpdate);
}

mu_run_tests(testSuite)
