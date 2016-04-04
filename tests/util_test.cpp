#include "awesomo/munit.h"
#include "awesomo/util.hpp"


// TESTS
int test_euler2Quaternion(void);
int test_deg2rad_and_rad2deg(void);
int test_euler2quat_and_quat2euler(void);
int test_euler2RotationMatrix(void);


int test_euler2Quaternion(void)
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    double x;
    double y;
    double z;
    double w;

    Eigen::Quaterniond q;

    euler2Quaternion(roll, pitch, yaw, q);
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();
    mu_check(fltcmp(x, 0.0) == 0);
    mu_check(fltcmp(y, 0.0) == 0);
    mu_check(fltcmp(z, 0.0) == 0);
    mu_check(fltcmp(w, 1.0) == 0);

    roll = M_PI/2;
    pitch = 0.0;
    yaw = 0.0;

    euler2Quaternion(roll, pitch, yaw, q);
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();

    mu_check(fltcmp(x, 0.707107) == 0);
    mu_check(fltcmp(y, 0.0) == 0);
    mu_check(fltcmp(z, 0.0) == 0);
    mu_check(fltcmp(w, 0.707107) == 0);

    roll = 0.0 ;
    pitch = M_PI;
    yaw = 0.0;

    euler2Quaternion(roll, pitch, yaw, q);
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();

    mu_check(fltcmp(x, 0.0) == 0);
    mu_check(fltcmp(y, 1.0) == 0);
    mu_check(fltcmp(z, 0.0) == 0);
    mu_check(fltcmp(w, 0.0) == 0);

    roll = 0.0 ;
    pitch = 0.0;
    yaw = M_PI/3;

    euler2Quaternion(roll, pitch, yaw, q);
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();

    mu_check(fltcmp(x, 0.0) == 0);
    mu_check(fltcmp(y, 0.0) == 0);
    mu_check(fltcmp(z, 0.5) == 0);
    mu_check(fltcmp(w, 0.866025) == 0);

    roll = M_PI ;
    pitch = M_PI/4;
    yaw = M_PI/3;

    euler2Quaternion(roll, pitch, yaw, q);
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();
    // std::cout << x << std::endl;
    // std::cout << y << std::endl;
    // std::cout << z << std::endl;
    // std::cout << w << std::endl;

    mu_check(fltcmp(x, 0.800103) == 0);
    mu_check(fltcmp(y, 0.46194) == 0);
    mu_check(fltcmp(z, 0.331414*-1) == 0);
    mu_check(fltcmp(w, 0.191342) == 0);

    return 0;
}

int test_euler2RotationMatrix(void)
{
    Eigen::Matrix3d rot;
    double roll;
    double pitch;
    double yaw;

    double r01, r02, r03;
    double r11, r12, r13;
    double r21, r22, r23;

    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    euler2RotationMatrix(roll, pitch, yaw, rot);

    // std::cout << rot << std::endl;
    r01 = 1;
    r02 = 0;
    r03 = 0;
    r11 = 0;
    r12 = 1;
    r13 = 0;
    r21 = 0;
    r22 = 0;
    r23 = 1;

    mu_check(fltcmp(r01,rot(0,0)) == 0);
    mu_check(fltcmp(r02,rot(0,1)) == 0);
    mu_check(fltcmp(r03,rot(0,2)) == 0);

    mu_check(fltcmp(r11,rot(1,0)) == 0);
    mu_check(fltcmp(r12,rot(1,1)) == 0);
    mu_check(fltcmp(r13,rot(1,2)) == 0);

    mu_check(fltcmp(r21,rot(2,0)) == 0);
    mu_check(fltcmp(r22,rot(2,1)) == 0);
    mu_check(fltcmp(r23,rot(2,2)) == 0);


    roll = M_PI;
    pitch = 0.0;
    yaw = 0.0;

    euler2RotationMatrix(roll, pitch, yaw, rot);
    // std::cout << rot << std::endl;
    r01 = 1;
    r02 = 0;
    r03 = 0;
    r11 = 0;
    r12 = -1;
    r13 = 0;
    r21 = 0;
    r22 = 0;
    r23 = -1;

    mu_check(fltcmp(r01,rot(0,0)) == 0);
    mu_check(fltcmp(r02,rot(0,1)) == 0);
    mu_check(fltcmp(r03,rot(0,2)) == 0);

    mu_check(fltcmp(r11,rot(1,0)) == 0);
    mu_check(fltcmp(r12,rot(1,1)) == 0);
    mu_check(fltcmp(r13,rot(1,2)) == 0);

    mu_check(fltcmp(r21,rot(2,0)) == 0);
    mu_check(fltcmp(r22,rot(2,1)) == 0);
    mu_check(fltcmp(r23,rot(2,2)) == 0);

    roll = M_PI;
    pitch = M_PI/2;
    yaw = 0.0;

    euler2RotationMatrix(roll, pitch, yaw, rot);
    // std::cout << rot << std::endl;
    r01 = 0;
    r02 = 0;
    r03 = -1;
    r11 = 0;
    r12 = -1;
    r13 = 0;
    r21 = -1;
    r22 = 0;
    r23 = 0;

    mu_check(fltcmp(r01,rot(0,0)) == 0);
    mu_check(fltcmp(r02,rot(0,1)) == 0);
    mu_check(fltcmp(r03,rot(0,2)) == 0);

    mu_check(fltcmp(r11,rot(1,0)) == 0);
    mu_check(fltcmp(r12,rot(1,1)) == 0);
    mu_check(fltcmp(r13,rot(1,2)) == 0);

    mu_check(fltcmp(r21,rot(2,0)) == 0);
    mu_check(fltcmp(r22,rot(2,1)) == 0);
    mu_check(fltcmp(r23,rot(2,2)) == 0);


    roll = M_PI;
    pitch = -M_PI/2;
    yaw = M_PI/3;

    euler2RotationMatrix(roll, pitch, yaw, rot);
    std::cout << rot << std::endl;
    r01 = 0;
    r02 = 0.866025;
    r03 = 0.5;
    r11 = 0;
    r12 = -0.5;
    r13 = 0.866025;
    r21 = 1;
    r22 = 0;
    r23 = 0;

    mu_check(fltcmp(r01,rot(0,0)) == 0);
    mu_check(fltcmp(r02,rot(0,1)) == 0);
    mu_check(fltcmp(r03,rot(0,2)) == 0);

    mu_check(fltcmp(r11,rot(1,0)) == 0);
    mu_check(fltcmp(r12,rot(1,1)) == 0);
    mu_check(fltcmp(r13,rot(1,2)) == 0);

    mu_check(fltcmp(r21,rot(2,0)) == 0);
    mu_check(fltcmp(r22,rot(2,1)) == 0);
    mu_check(fltcmp(r23,rot(2,2)) == 0);
    return 0;


}




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
    mu_add_test(test_euler2Quaternion);
    mu_add_test(test_euler2RotationMatrix);
    mu_add_test(test_deg2rad_and_rad2deg);
    mu_add_test(test_euler2quat_and_quat2euler);
}

mu_run_tests(test_suite)
