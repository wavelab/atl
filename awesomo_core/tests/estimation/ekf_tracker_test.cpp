#include <random>
#include <fstream>
#include <iostream>

#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/estimation/ekf_tracker.hpp"

#define TEST_CONFIG "tests/configs/estimation/ekf_tracker.yaml"
#define TEST_CONFIG2 "tests/configs/estimation/ekf_tracker2.yaml"
#define TEST_OUTPUT_FILE "/tmp/estimation_ekf_tracker_test.output"


namespace awesomo {

static int prepareOutputFile(std::ofstream &output_file,
                             std::string output_path) {
  // clang-format off
  output_file.open(output_path);

  output_file << "time_step" << ",";

  output_file << "x" << ",";
  output_file << "y" << ",";
  output_file << "z" << ",";

  output_file << "bx" << ",";
  output_file << "by" << ",";
  output_file << "bz" << std::endl;
  // clang-format on

  return 0;
}

static void recordTimeStep(std::ofstream &output_file,
                           int i,
                           VecX mea,
                           VecX est) {
  // record true state x, y, z
  output_file << i << ",";
  output_file << mea(0) << ",";
  output_file << mea(1) << ",";
  output_file << mea(2) << ",";

  // record belief state x, y, z
  output_file << est(0) << ",";
  output_file << est(1) << ",";
  output_file << est(2) << std::endl;
}

TEST(ExtendedKalmanFilterTracker, estimate) {
  float dt;
  Vec2 u;
  Vec3 mu, x, y, g, h, gaussian_noise;
  Mat3 G, H;
  ExtendedKalmanFilterTracker tracker;
  std::ofstream output_file;
  std::default_random_engine rgen;
  std::normal_distribution<float> norm_x(0, pow(0.5, 2));
  std::normal_distribution<float> norm_y(0, pow(0.5, 2));
  std::normal_distribution<float> norm_theta(0, pow(deg2rad(0.5), 2));

  // setup
  // clang-format off
  dt = 0.01;
  x << 0, 0, 0;
  mu << 0.0, 0.0, 0.0;    // x, y, theata
  u << 1.0, 0.1;
  tracker.configure(TEST_CONFIG);
  tracker.initialize(mu);
  prepareOutputFile(output_file, TEST_OUTPUT_FILE);
  // clang-format on

  // estimate
  for (int i = 0; i < 10000; i++) {
    // update true state
    // clang-format off
    x << x(0) + u(0) * cos(x(2)) * dt,
         x(1) + u(0) * sin(x(2)) * dt,
         x(2) + u(1) * dt;
    // clang-format on

    // take measurement
    gaussian_noise << norm_x(rgen), norm_y(rgen), norm_theta(rgen);
    y = x + gaussian_noise;

    // propagate motion model
    TWO_WHEEL_MOTION_MODEL(tracker, G, g);
    tracker.predictionUpdate(g, G);

    // propagate measurement
    TWO_WHEEL_MEASUREMENT_MODEL(tracker, H, h);
    tracker.measurementUpdate(h, H, y);

    // record
    recordTimeStep(output_file, i, x, tracker.mu);
  }
  output_file.close();
}

TEST(ExtendedKalmanFilterTracker, estimate2) {
  float dt;
  VecX u(2), mu(5), x(5), y(5), g(5), h(5), gaussian_noise(5);
  MatX G(5, 5), H(5, 5);
  ExtendedKalmanFilterTracker tracker;
  std::ofstream output_file;
  std::default_random_engine rgen;
  std::normal_distribution<float> norm_x(0, pow(0.5, 2));
  std::normal_distribution<float> norm_y(0, pow(0.5, 2));
  std::normal_distribution<float> norm_theta(0, pow(deg2rad(0.5), 2));
  std::normal_distribution<float> pn1(0, pow(0.5, 2));
  std::normal_distribution<float> pn2(0, pow(0.5, 2));

  // setup
  // clang-format off
  dt = 0.01;
  x << 0, 0, 0, 1.0, 0.1;
  mu << 0, 0, 0, 1.0, 0.1;  // x, y, theata, omega, v
  u << 1.0, 0.1;
  tracker.configure(TEST_CONFIG2);
  tracker.initialize(mu);
  prepareOutputFile(output_file, TEST_OUTPUT_FILE);
  // clang-format on

  // estimate
  for (int i = 0; i < 6000; i++) {
    // update true state
    // clang-format off
    x << x(0) + u(0) * cos(x(2)) * dt,
         x(1) + u(0) * sin(x(2)) * dt,
         x(2) + u(1) * dt,
         u(0),
         u(1);
    // clang-format on

    // take measurement
    gaussian_noise << norm_x(rgen), norm_y(rgen), norm_theta(rgen), 0.0, 0.0;
    y = x + gaussian_noise;

    // propagate motion model
    TWO_WHEEL_NO_INPUTS_MOTION_MODEL(tracker, G, g, pn1(rgen), pn2(rgen));
    tracker.predictionUpdate(g, G);

    // propagate measurement
    TWO_WHEEL_NO_INPUTS_MEASUREMENT_MODEL(tracker, H, h);
    tracker.measurementUpdate(h, H, y);

    // record
    recordTimeStep(output_file, i, x, tracker.mu);
  }
  output_file.close();
}

}  // end of awesomo namespace
