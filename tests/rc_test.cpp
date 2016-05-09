#include <unistd.h>

#include "munit.h"
#include "rc.hpp"


// TESTS
int testRC(void);


int testRC(void)
{
    RCControl rc;

    while (1) {
        rc.update();
        rc.print();
        sleep(1);
    }

	return 0;
}

void testSuite(void)
{
    mu_add_test(testRC);
}

mu_run_tests(testSuite)
