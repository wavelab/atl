#include <iostream>
#include <random>

#include "awesomo_core/munit.h"
#include "awesomo_core/estimator.hpp"


// TESTS
int test_apriltag_kf_setup(void);
int test_apriltag_kf_estimate(void);

int test_apriltag_kf_setup(void)
{
    struct kf estimator;
    Eigen::VectorXd mu(9);

    mu << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    apriltag_kf_setup(&estimator, mu);

    return 0;
}

int test_apriltag_kf_estimate(void)
{
  float dt;
  bool tag_detected;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    struct kf estimator;
    Eigen::VectorXd state(9);
    Eigen::VectorXd mu(9);
    Eigen::VectorXd y(3);
    Eigen::VectorXd motion_noise(3);
    std::ofstream test_data;
  std::default_random_engine generator;
  std::normal_distribution<float> norm_dist_x(0, 0.5);
  std::normal_distribution<float> norm_dist_y(0, 0.5);
  std::normal_distribution<float> norm_dist_z(0, 0.5);

  // setup
  dt = 0.01;
  pos << 0, 0, 0;
  vel << 9, 30, 0;
  acc << 0, -10, 0;
    mu << 0.0, 0.0, 0.0,    // x, y, z
      9.0, 30.0, 0.0,   // x_dot, y_dot, z_dot
      0.0, -10.0, 0.0;  // x_ddot, y_ddot, z_ddot
    apriltag_kf_setup(&estimator, mu);

  // prepare test data file
    test_data.open("estimator_test.dat");
  test_data << "time_step" << ",";
  test_data << "x" << ",";
  test_data << "y" << ",";
  test_data << "z" << ",";
  test_data << "bx" << ",";
  test_data << "by" << ",";
  test_data << "bz" << std::endl;

  // estimate
  for (int i = 0; i < 1000; i++) {
    // update true state
    vel = vel + acc * dt;
    pos = pos + vel * dt;
    state << pos(0), pos(1), pos(2),
         vel(0), vel(1), vel(2),
         acc(0), acc(1), acc(2);

    // perform measurement
    motion_noise << norm_dist_x(generator),
              norm_dist_y(generator),
              norm_dist_z(generator);
    y = estimator.C * state + motion_noise;

    // estimate
    if (i % 10 == 0) {
      tag_detected = true;
    } else {
      tag_detected = false;
    }
    apriltag_kf_estimate(&estimator, y, dt, tag_detected);

    // record true state x, y, z
    test_data << i << ",";
    test_data << pos(0) << ",";
    test_data << pos(1) << ",";
    test_data << pos(2) << ",";
    // record belief state x, y, z
    test_data << estimator.mu(0) << ",";
    test_data << estimator.mu(1) << ",";
    test_data << estimator.mu(2) << std::endl;
  }
    test_data.close();

  return 0;
}

void test_suite(void)
{
    mu_add_test(test_apriltag_kf_setup);
    mu_add_test(test_apriltag_kf_estimate);
}

mu_run_tests(test_suite)
