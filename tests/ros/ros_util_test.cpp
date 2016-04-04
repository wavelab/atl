#include "awesomo/munit.h"
#include "awesomo/ros/ros_util.hpp"

// TESTS
int test_rotation_matrix(void);
int test_single_rotation_matrx(void);
int test_mat3_dot_vec3(void);
int test_fix_coordinate_frames(void);
int test_build_pose_stamped_msg(void);
int test_build_pose_cov_stamped_msg(void);
int test_print_pose_stamped_msg(void);
int test_print_pose_cov_stamped_msg(void);
void test_suite(void);


int test_rotation_matrix(void)
{
    double rot_mat[9];
    double phi;
    double theta;
    double psi;

    // setup
    phi = 0.0;
    theta = 0.0;
    psi = 0.0;

    // test and assert
    rotation_matrix(phi, theta, psi, rot_mat);
    std::cout << rot_mat[0] << std::endl;
    mu_check(fltcmp(rot_mat[0], 0.0) == 0);
    mu_check(fltcmp(rot_mat[1], 0.0) == 0);
    mu_check(fltcmp(rot_mat[2], 0.0) == 0);
    mu_check(fltcmp(rot_mat[3], 0.0) == 0);
    mu_check(fltcmp(rot_mat[4], 0.0) == 0);
    mu_check(fltcmp(rot_mat[5], 0.0) == 0);
    mu_check(fltcmp(rot_mat[6], 0.0) == 0);
    mu_check(fltcmp(rot_mat[7], 0.0) == 0);
    mu_check(fltcmp(rot_mat[8], 0.0) == 0);

    return 0;
}

int test_single_rotation_matrx(void)
{
    double rot_mat[9];
    double phi;
    double theta;
    double psi;

    // setup
    phi = 0.0;
    theta = 0.0;
    psi = 0.0;

    // test and assert
    single_rotation_matrix(psi, rot_mat);
    mu_check(fltcmp(rot_mat[0], 0.0) == 0);
    mu_check(fltcmp(rot_mat[1], 0.0) == 0);
    mu_check(fltcmp(rot_mat[2], 0.0) == 0);
    mu_check(fltcmp(rot_mat[3], 0.0) == 0);
    mu_check(fltcmp(rot_mat[4], 0.0) == 0);
    mu_check(fltcmp(rot_mat[5], 0.0) == 0);
    mu_check(fltcmp(rot_mat[6], 0.0) == 0);
    mu_check(fltcmp(rot_mat[7], 0.0) == 0);
    mu_check(fltcmp(rot_mat[8], 0.0) == 0);

    return 0;
}

int test_mat3_dot_vec3(void)
{
    int i;
    double mat[9];
    double v_in[3];
    double v_out[3];
    double solution[3];

    /* matrix 1 */
    mat[0] = 2.0f;
    mat[3] = 4.0f;
    mat[6] = 6.0f;

    mat[1] = 8.0f;
    mat[4] = 10.0f;
    mat[7] = 12.0f;

    mat[2] = 14.0f;
    mat[5] = 16.0f;
    mat[8] = 18.0f;

    /* vector */
    v_in[0] = 1.0f;
    v_in[1] = 2.0f;
    v_in[2] = 3.0f;

    /* solution vector */
    solution[0] = 28.0f;
    solution[1] = 64.0f;
    solution[2] = 100.0f;

    /* assert tests */
    mat3_dot_vec3(mat, v_in, v_out);
    for (i = 0; i < 3; i++) {
        mu_check(fltcmp(v_out[i], solution[i]) == 0);
    }

    return 0;
}

int test_fix_coordinate_frames(void)
{

    return 0;
}

int test_build_pose_stamped_msg(void)
{

    return 0;
}

int test_build_pose_cov_stamped_msg(void)
{

    return 0;
}


void test_suite(void)
{
    mu_add_test(test_rotation_matrix);
    // mu_add_test(test_rotation_matrix);
    // mu_add_test(test_single_rotation_matrx);
    // mu_add_test(test_mat3_dot_vec3);
    // mu_add_test(test_fix_coordinate_frames);
    // mu_add_test(test_build_pose_stamped_msg);
    // mu_add_test(test_build_pose_cov_stamped_msg);
    // mu_add_test(test_print_pose_stamped_msg);
    //mu_add_test(test_print_pose_cov_stamped_msg);
}

mu_run_tests(test_suite)
