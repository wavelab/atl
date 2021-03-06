#include "atl/planning/model.hpp"
#include "atl/atl_test.hpp"

#define SIM_OUTPUT "/tmp/sim.output"

namespace atl {

TEST(Quad2DModel, constructor) {
  Quad2DModel model;

  EXPECT_FALSE(model.configured);
  EXPECT_FLOAT_EQ(0.0, model.x(0, 0));
  EXPECT_FLOAT_EQ(0.0, model.m);
}

TEST(Quad2DModel, configure) {
  Quad2DModel model;
  Vec4 x_init;

  x_init << 1.0, 2.0, 3.0, 4.0;
  model.configure(x_init, 1.0);

  EXPECT_TRUE(model.configured);
  EXPECT_FLOAT_EQ(1.0, model.x(0));
  EXPECT_FLOAT_EQ(2.0, model.x(1));
  EXPECT_FLOAT_EQ(3.0, model.x(2));
  EXPECT_FLOAT_EQ(4.0, model.x(3));
  EXPECT_FLOAT_EQ(1.0, model.m);
}

TEST(Quad2DModel, update) {
  Quad2DModel model;
  Vec2 u;
  Vec4 x_init;

  x_init << 0.0, 0.0, 0.0, 0.0;
  u << 10.0, deg2rad(0.0);

  model.configure(x_init, 1.0);
  model.printState();
  std::cout << std::endl;

  model.update(u, 1.0);
  model.printState();
  std::cout << std::endl;

  model.update(u, 1.0);
  model.printState();
  std::cout << std::endl;
}

TEST(Simulator, constructor) {
  Simulator sim;

  EXPECT_FALSE(sim.configured);
  EXPECT_FALSE(sim.model.configured);

  EXPECT_FLOAT_EQ(0.0, sim.x_init(0));
  EXPECT_FLOAT_EQ(0.0, sim.x_init(1));
  EXPECT_FLOAT_EQ(0.0, sim.x_init(2));
  EXPECT_FLOAT_EQ(0.0, sim.x_init(3));

  EXPECT_FLOAT_EQ(0.0, sim.x_final(0));
  EXPECT_FLOAT_EQ(0.0, sim.x_final(1));
  EXPECT_FLOAT_EQ(0.0, sim.x_final(2));
  EXPECT_FLOAT_EQ(0.0, sim.x_final(3));
}

TEST(Simulator, configure) {
  Simulator sim;
  Vec4 x_init;
  Vec4 x_final;
  double m;

  // test
  x_init << 1.0, 2.0, 3.0, 4.0;
  x_final << 11.0, 12.0, 13.0, 14.0;
  m = 1.0;
  sim.configure(x_init, x_final, m);

  // assert
  EXPECT_TRUE(sim.configured);
  EXPECT_TRUE(sim.model.configured);

  EXPECT_FLOAT_EQ(1.0, sim.x_init(0));
  EXPECT_FLOAT_EQ(2.0, sim.x_init(1));
  EXPECT_FLOAT_EQ(3.0, sim.x_init(2));
  EXPECT_FLOAT_EQ(4.0, sim.x_init(3));

  EXPECT_FLOAT_EQ(11.0, sim.x_final(0));
  EXPECT_FLOAT_EQ(12.0, sim.x_final(1));
  EXPECT_FLOAT_EQ(13.0, sim.x_final(2));
  EXPECT_FLOAT_EQ(14.0, sim.x_final(3));
}

// TEST(Simulator, simulate) {
//   int retval;
//   double m;
//   Simulator sim;
//   Vec4 x_init, x_final;
//   MatX U, X;
//   Vec2 u;
//   std::ofstream output_file;
//
//   // test
//   x_init << 1.0, 0.0, 5.0, 0.0;
//   x_final << 1.0, 0.0, 5.0, 0.0;
//   m = 1.0;
//
//   U.resize(2, 10);
//   u << 10.0, deg2rad(10.0);
//   for (int i = 0; i < 10; i++) {
//     U.block(0, i, 2, 1) = u;
//   }
//
//   sim.configure(x_init, x_final, m);
//   retval = sim.simulate(0.1, 1.0, U, X);
//
//   // ouput file
//   output_file.open(SIM_OUTPUT);
//   output_file << "time_step"
//               << ",";
//   output_file << "x"
//               << ",";
//   output_file << "vx"
//               << ",";
//   output_file << "z"
//               << ",";
//   output_file << "vz"
//               << ",";
//   output_file << "theta" << std::endl;
//
//   for (int i = 0; i < 10; i++) {
//     output_file << i << ",";
//     output_file << X(0, i) << ",";
//     output_file << X(1, i) << ",";
//     output_file << X(2, i) << ",";
//     output_file << X(3, i) << ",";
//     output_file << U(1, i) << "\n";
//   }
//
//   output_file.close();
//
//   // asssert
//   EXPECT_EQ(0, retval);
//   EXPECT_FLOAT_EQ(0.0, sim.d_az);
//   EXPECT_FLOAT_EQ(0.0, sim.d_theta);
//   EXPECT_FLOAT_EQ(10.0, sim.az_sum);
//   EXPECT_TRUE(sim.dist_error > 0);
//   EXPECT_TRUE(sim.vel_error > 0);
// }

} // namespace atl
