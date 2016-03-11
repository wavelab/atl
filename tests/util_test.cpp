#include "awesomo/munit.h"
#include "awesomo/util.hpp"


// TESTS
int test_deg2rad_and_rad2deg(void);
int test_euler2quat_and_quat2euler(void);


int test_deg2rad_and_rad2deg(void)
{
    double d_deg;
    double d_rad;

    d_deg = 10;
    d_rad = deg2rad(d_deg);
    mu_check(fltcmp(rad2deg(d_rad), d_deg) == 0);

    return 0;
}

int test_euler2quat_and_quat2euler(void)
{
    double roll;
    double pitch;
    double yaw;
    tf::Quaternion tf_q;
    geometry_msgs::Quaternion geo_q;

    tf_q = euler2quat(
        deg2rad(10),
        deg2rad(20),
        deg2rad(30)
    );
    geo_q.x = tf_q.x();
    geo_q.y = tf_q.y();
    geo_q.z = tf_q.z();
    geo_q.w = tf_q.w();

    quat2euler(geo_q, &roll, &pitch, &yaw);
    mu_check(fltcmp(roll, deg2rad(10)) == 0);
    mu_check(fltcmp(pitch, deg2rad(20)) == 0);
    mu_check(fltcmp(yaw, deg2rad(30)) == 0);

    return 0;
}


void test_suite(void)
{
    mu_add_test(test_deg2rad_and_rad2deg);
    mu_add_test(test_euler2quat_and_quat2euler);
}

mu_run_tests(test_suite)
