#include <unistd.h>

#include "munit.h"
#include "rc.hpp"
#include "motor.hpp"


// TESTS
int test_set_throttle(void);


int test_set_throttle(void)
{
    Motors *motors;

    motors = new Motors();

    motors->arm();
    motors->set_throttle(0, 1);
    motors->set_throttle(1, 1);
    motors->set_throttle(2, 1);
    motors->set_throttle(3, 1);
    sleep(2);

    motors->set_throttle(0, 0);
    motors->set_throttle(1, 0);
    motors->set_throttle(2, 0);
    motors->set_throttle(3, 0);
    sleep(2);

    // motors->set_throttle(0, 0.1);
    // sleep(1);
    // motors->set_throttle(1, 0.1);
    // sleep(1);
    // motors->set_throttle(2, 0.1);
    // sleep(1);
    // motors->set_throttle(3, 0.1);
    // sleep(1);
    //
    // motors->set_throttle(0, 0);
    // motors->set_throttle(1, 0);
    // motors->set_throttle(2, 0);
    // motors->set_throttle(3, 0);
    // sleep(2);

	return 0;
}

int test_throttle_with_rc(void)
{
    Motors *motors;
    RCControl *rc_control;
    float roll;
    float pitch;
    float throttle;

    motors = new Motors();
    rc_control = new RCControl();

    while (1) {
        rc_control->update();

        roll = (rc_control->ch1 - 1500.0) / (2000.0 - 1500.0);
        pitch = (rc_control->ch2 - 1500.0) / (2000.0 - 1500.0);
        throttle = (rc_control->ch3 - 1092) / (1900.0 - 1092.0);

        motors->set_throttle(0, throttle - roll + pitch);
        motors->set_throttle(1, throttle + roll - pitch);
        motors->set_throttle(2, throttle + roll + pitch);
        motors->set_throttle(3, throttle - roll - pitch);
    }

    return 0;
}

void testSuite(void)
{
    mu_add_test(test_set_throttle);
    // mu_add_test(test_throttle_with_rc);
}

mu_run_tests(testSuite)
