#include <unistd.h>

#include "awesomo/munit.h"
#include "awesomo/pid.hpp"


/* TESTS */
int test_pid_setup_and_destroy(void);
int test_pid_calculate(void);
void test_suite(void);


int test_pid_setup_and_destroy(void)
{
    struct pid *p;
    struct pid_config c;

    /* setup piq config */
    c.sample_rate = 100;
    c.setpoint = 0.0f;
    c.k_p = 0.0f;
    c.k_i = 0.0f;
    c.k_d = 0.0f;
    c.dead_zone = 0.0f;
    c.min = 0.0f;
    c.max = 0.0f;

    /* setup piq */
    p = pid_setup(&c);
    mu_check(fltcmp(p->setpoint, 0.0f) == 0);
    mu_check(fltcmp(p->k_p, 0.0f) == 0);
    mu_check(fltcmp(p->k_i, 0.0f) == 0);
    mu_check(fltcmp(p->k_d, 0.0f) == 0);
    mu_check(fltcmp(p->dead_zone, 0.0f) == 0);
    mu_check(fltcmp(p->min, 0.0f) == 0);
    mu_check(fltcmp(p->max, 0.0f) == 0);
    pid_destroy(p);

    return 0;
}

int test_pid_calculate(void)
{
    int i;
    int retval;
    struct pid *p;
    struct pid_config c;

    /* setup piq config */
    c.sample_rate = 100;
    c.setpoint = 0.0f;
    c.k_p = 0.1f;
    c.k_i = 0.0f;
    c.k_d = 0.1f;
    c.dead_zone = 0.0f;
    c.min = -1.0f;
    c.max = 1.0f;

    /* test */
    p = pid_setup(&c);
    p->output = 1.0f;
    for (i = 0; i < 10; i++) {
        usleep(100000);
        retval = pid_calculate(p, p->output);
        mu_check(retval == 0);
        std::cout << "p->output: " << p->output << std::endl;
    }
    pid_destroy(p);

    return 0;
}

void test_suite(void)
{
    mu_add_test(test_pid_setup_and_destroy);
    mu_add_test(test_pid_calculate);
}

mu_run_tests(test_suite)
