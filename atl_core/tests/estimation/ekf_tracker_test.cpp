#include <random>
#include <fstream>
#include <iostream>

#include "atl/atl_test.hpp"
#include "atl/estimation/ekf_tracker.hpp"

#define TEST_CONFIG "tests/configs/estimation/ekf_tracker.yaml"
#define TEST_CONFIG2 "tests/configs/estimation/ekf_tracker2.yaml"
#define TEST_CONFIG3 "tests/configs/estimation/ekf_tracker3.yaml"
#define TEST_OUTPUT_FILE "/tmp/estimation_ekf_tracker_test.output"


namespace atl {

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

// TEST(ExtendedKalmanFilterTracker, estimate3) {
//   float dt;
//   VecX u(3), mu(7), x(7), y(4), g(7), h(4), gaussian_noise(4);
//   MatX G(7, 7), H(4, 7);
//   ExtendedKalmanFilterTracker tracker;
//   std::ofstream output_file;
//   std::default_random_engine rgen;
//   std::normal_distribution<float> norm_x(0, pow(0.2, 2));
//   std::normal_distribution<float> norm_y(0, pow(0.2, 2));
//   std::normal_distribution<float> norm_z(0, pow(0.01, 2));
//   std::normal_distribution<float> norm_theta(0, pow(deg2rad(0.5), 2));
//
//   // setup
//   // clang-format off
//   dt = 0.01;
//   x << 0, 0, 0, 0.0, 0.1, 0.0, 0.0;
//   mu << 0, 0, 0, 0.0, 0.1, 0.0, 0.0;  // x, y, z, theata, v, omega, vz
//   u << 1.0, 0.1, 0.0;
//   tracker.configure(TEST_CONFIG3);
//   tracker.initialize(mu);
//   prepareOutputFile(output_file, TEST_OUTPUT_FILE);
//   // clang-format on
//
//   // estimate
//   for (int i = 0; i < 6000; i++) {
//     // update true state
//     // clang-format off
//     x << x(0) + u(0) * cos(x(3)) * dt,
//          x(1) + u(0) * sin(x(3)) * dt,
//          x(2) + u(2) * dt,
//          x(3) + u(1) * dt,
//          u(0),
//          u(1),
//          u(2);
//     // clang-format on
//
//     // take measurement
//     // clang-format off
//     gaussian_noise << norm_x(rgen),
//                       norm_y(rgen),
//                       norm_z(rgen),
//                       norm_theta(rgen);
//     y = x.block(0, 0, 4, 1) + gaussian_noise;
//     // clang-format on
//
//     // propagate motion model
//     TWO_WHEEL_3D_NO_INPUTS_MOTION_MODEL(tracker, G, g);
//     tracker.predictionUpdate(g, G);
//
//     // propagate measurement
//     TWO_WHEEL_3D_NO_INPUTS_MEASUREMENT_MODEL(tracker, H, h);
//     tracker.measurementUpdate(h, H, y);
//
//     // record
//     recordTimeStep(output_file, i, x, tracker.mu);
//   }
//   output_file.close();
// }

TEST(ExtendedKalmanFilterTracker, estimate3) {
  float dt;
  VecX u(3), mu(9), x(9), y(4), g(9), h(4), gaussian_noise(4);
  MatX G(9, 9), H(4, 9);
  ExtendedKalmanFilterTracker tracker;
  std::ofstream output_file;
  std::default_random_engine rgen;
  std::normal_distribution<float> norm_x(0, pow(0.5, 2));
  std::normal_distribution<float> norm_y(0, pow(0.5, 2));
  std::normal_distribution<float> norm_z(0, pow(0.5, 2));
  std::normal_distribution<float> norm_theta(0, pow(deg2rad(10), 2));

  // x0 - x
  // x1 - y
  // x2 - z
  // x3 - theta
  // x4 - v
  // x5 - vz
  // x6 - omega
  // x7 - a
  // x8 - az

  // setup
  // clang-format off
  dt = 0.01;
  x << 0, 0, 0,
       0, 1.0, 0,
       0, 0, 0;
  mu << 0, 0, 0,
       0, 1.0, 0,
       0, 0, 0;
  u << 0.0, 0.0, 0.0;

  tracker.configure(TEST_CONFIG3);
  tracker.initialize(mu);
  prepareOutputFile(output_file, TEST_OUTPUT_FILE);
  // clang-format on

  // estimate
  for (int i = 0; i < 1000; i++) {
    // update true state
    // clang-format off
    x(0) = x(0) + x(4) * cos(x(3)) * dt;
    x(1) = x(1) + x(4) * sin(x(3)) * dt;
    x(2) = x(2) + x(4) * dt;
    x(3) = x(3) + u(0) * dt;
    x(4) = x(4) + x(7) * dt;
    x(5) = x(5) + x(8) * dt;
    x(6) = u(0);
    x(7) = u(1);
    x(8) = u(2);
    // clang-format on

    // take measurement
    // clang-format off
    gaussian_noise << norm_x(rgen),
                      norm_y(rgen),
                      norm_z(rgen),
                      norm_theta(rgen);
    y = x.block(0, 0, 4, 1) + gaussian_noise;
    // clang-format on

    // propagate motion model
    two_wheel_process_model(tracker, G, g, dt);
    tracker.predictionUpdate(g, G);

    // propagate measurement
    H = MatX::Zero(4, 9);
    if (i % 3 == 0) {
      H(0, 0) = 1.0; /* x */
      H(1, 1) = 1.0; /* y */
      H(2, 2) = 1.0; /* z */
      H(3, 3) = 1.0; /* theta */
      h = H * tracker.mu_p;
      tracker.measurementUpdate(h, H, y);
    } else {
      tracker.mu = tracker.mu_p;
    }

    // record true state x, y, z
    output_file << i << ",";
    output_file << x(0) << ",";
    output_file << x(1) << ",";
    output_file << x(2) << ",";
    output_file << x(3) << ",";
    output_file << x(4) << ",";

    // record belief state x, y, z
    output_file << tracker.mu(0) << ",";
    output_file << tracker.mu(1) << ",";
    output_file << tracker.mu(2) << ",";
    output_file << tracker.mu(3) << ",";
    output_file << tracker.mu(4) << std::endl;
  }
  output_file.close();
}

}  // namespace atl
