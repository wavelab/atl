#include "awesomo/munit.h"
#include "awesomo/sim.hpp"


// TESTS
int test_quadrotor_rotation_matrix(void);
int test_quadrotor_inertia_matrix(void);
int test_quadrotor_calculate_thrust(void);
int test_quadrotor_calculate_drag(void);
int test_quadrotor_calculate_torque(void);
int test_quadrotor_calculate_acceleration(void);
int test_quadrotor_convert_angular_velocity_to_body_frame(void);
int test_quadrotor_convert_angular_velocity_to_inertial_frame(void);
int test_quadrotor_calculate_angular_acceleration(void);
int test_sim_loop(void);


int test_quadrotor_rotation_matrix(void)
{
    struct quadrotor q;
    Eigen::Matrix3d m;

    // setup
    q.orientation << 1.0f, 1.0f, 1.0f;

    // test and assert
    quadrotor_rotation_matrix(&q, m);
    mu_check(fltcmp(m(0), 0.291927) == 0);
    mu_check(fltcmp(m(1), 0.454649) == 0);
    mu_check(fltcmp(m(2), -0.841471) == 0);

    mu_check(fltcmp(m(3), -0.072075) == 0);
    mu_check(fltcmp(m(4), 0.887750) == 0);
    mu_check(fltcmp(m(5), 0.454649) == 0);

    mu_check(fltcmp(m(6), 0.953721) == 0);
    mu_check(fltcmp(m(7), -0.072075) == 0);
    mu_check(fltcmp(m(8), 0.291927) == 0);

    return 0;
}

int test_quadrotor_inertia_matrix(void)
{
    struct quadrotor q;

    quadrotor_inertia_matrix(&q, 1.0, 2.0, 3.0);

    mu_check(fltcmp(q.inertia(0), 1.0) == 0);
    mu_check(fltcmp(q.inertia(1), 0.0) == 0);
    mu_check(fltcmp(q.inertia(2), 0.0) == 0);

    mu_check(fltcmp(q.inertia(3), 0.0) == 0);
    mu_check(fltcmp(q.inertia(4), 2.0) == 0);
    mu_check(fltcmp(q.inertia(5), 0.0) == 0);

    mu_check(fltcmp(q.inertia(6), 0.0) == 0);
    mu_check(fltcmp(q.inertia(7), 0.0) == 0);
    mu_check(fltcmp(q.inertia(8), 3.0) == 0);

    return 0;
}

int test_quadrotor_calculate_thrust(void)
{
    struct quadrotor q;

    // setup
    q.orientation << 0.0f, 0.0f, 0.0f;
    q.rotors << 10.0f, 10.0f, 10.0f, 10.0f;

    // test and assert
    quadrotor_calculate_thrust(&q);
    mu_check(fltcmp(q.thrust(0), 0.0f) == 0);
    mu_check(fltcmp(q.thrust(1), 0.0f) == 0);
    mu_check(fltcmp(q.thrust(2), 400.0f) == 0);

    return 0;
}

int test_quadrotor_calculate_drag(void)
{
    struct quadrotor q;

    // setup
    q.kd = 1.0f;
    q.velocity << 10.0f, 20.0f, 30.0f;

    // test and assert
    quadrotor_calculate_drag(&q);
    mu_check(fltcmp(q.drag(0), -10.0f) == 0);
    mu_check(fltcmp(q.drag(1), -20.0f) == 0);
    mu_check(fltcmp(q.drag(2), -30.0f) == 0);

    return 0;
}

int test_quadrotor_calculate_torque(void)
{
    struct quadrotor q;

    // setup
    q.L = 1.0f;
    q.k = 1.0f;
    q.b = 1.0f;
    q.rotors << 1.0f, 2.0f, 3.0f, 4.0f;

    // test and assert
    quadrotor_calculate_torque(&q);
    mu_check(fltcmp(q.torque(0), -8.0f) == 0);
    mu_check(fltcmp(q.torque(1), -12.0f) == 0);
    mu_check(fltcmp(q.torque(2), -10.0f) == 0);

    return 0;
}

int test_quadrotor_calculate_acceleration(void)
{
    struct quadrotor q;

    // setup
    q.m = 1.0f;
    q.L = 1.0f;
    q.k = 1.0f;
    q.b = 1.0f;
    q.kd = 1.0f;
    q.orientation << 0.0f, 0.0f, 0.0f;
    q.rotors << 10.0f, 10.0f, 10.0f, 10.0f;
    q.velocity << 10.0f, 10.0f, 10.0f;

    // test and assert
    quadrotor_calculate_acceleration(&q, 10.0f);

    return 0;
}

int test_quadrotor_convert_angular_velocity_to_body_frame(void)
{
    struct quadrotor q;

    // setup
    q.orientation(0) = deg2rad(10);
    q.orientation(1) = deg2rad(20);
    q.orientation(2) = deg2rad(30);

    q.angular_velocity(0) = 1.0f;
    q.angular_velocity(1) = 1.0f;
    q.angular_velocity(2) = 1.0f;

    // test and assert
    quadrotor_convert_angular_velocity_to_body_frame(&q);
    mu_check(fltcmp(q.angular_velocity_body_frame(0), 1.0) == 0);
    mu_check(fltcmp(q.angular_velocity_body_frame(1), 0.36603) == 0);
    mu_check(fltcmp(q.angular_velocity_body_frame(2), 0.94162) == 0);

    return 0;
}

int test_quadrotor_convert_angular_velocity_to_inertial_frame(void)
{
    struct quadrotor q;

    // setup
    q.orientation(0) = deg2rad(10);
    q.orientation(1) = deg2rad(20);
    q.orientation(2) = deg2rad(30);

    q.angular_velocity_body_frame(0) = 1.0;
    q.angular_velocity_body_frame(1) = 0.36603;
    q.angular_velocity_body_frame(2) = 0.94162;

    // test and assert
    quadrotor_convert_angular_velocity_to_inertial_frame(&q);
    mu_check(fltcmp(q.angular_velocity(0), 1.0f) == 0);
    mu_check(fltcmp(q.angular_velocity(1), 1.0f) == 0);
    mu_check(fltcmp(q.angular_velocity(2), 1.0f) == 0);

    return 0;
}

int test_quadrotor_calculate_angular_acceleration(void)
{
    struct quadrotor q;

    // setup
    q.L = 1.0f;
    q.k = 1.0f;
    q.b = 1.0f;
    q.rotors << 1.0f, 2.0f, 3.0f, 4.0f;
    quadrotor_inertia_matrix(&q, 1.0f, 1.0f, 1.0f);
    q.angular_velocity_body_frame << 1.0f, 1.0f, 1.0f;

    // test and assert
    quadrotor_calculate_angular_acceleration(&q);
    mu_check(fltcmp(q.torque(0), -8.0f) == 0);
    mu_check(fltcmp(q.torque(1), -12.0f) == 0);
    mu_check(fltcmp(q.torque(2), -10.0f) == 0);

    return 0;
}

int test_sim_loop(void)
{
    sim_loop();
    return 0;
}

void test_suite(void)
{
    mu_add_test(test_quadrotor_rotation_matrix);
    mu_add_test(test_quadrotor_inertia_matrix);
    mu_add_test(test_quadrotor_calculate_thrust);
    mu_add_test(test_quadrotor_calculate_drag);
    mu_add_test(test_quadrotor_calculate_torque);
    mu_add_test(test_quadrotor_calculate_acceleration);
    // mu_add_test(test_quadrotor_convert_angular_velocity_to_body_frame);
    // mu_add_test(test_quadrotor_convert_angular_velocity_to_inertial_frame);
    mu_add_test(test_quadrotor_calculate_angular_acceleration);
    mu_add_test(test_sim_loop);
}

mu_run_tests(test_suite)
