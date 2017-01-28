#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/model.hpp"

namespace awesomo {

TEST(Quad2DModel, constructor) {
  Quad2DModel model;

  ASSERT_FALSE(model.configured);
  ASSERT_FLOAT_EQ(0.0, model.x(0, 0));
  ASSERT_FLOAT_EQ(0.0, model.m);
}

TEST(Quad2DModel, configure) {
  Quad2DModel model;
  Vec4 x_init;

  x_init << 1.0, 2.0, 3.0, 4.0;
  model.configure(x_init, 1.0);

  ASSERT_TRUE(model.configured);
  ASSERT_FLOAT_EQ(1.0, model.x(0));
  ASSERT_FLOAT_EQ(2.0, model.x(1));
  ASSERT_FLOAT_EQ(3.0, model.x(2));
  ASSERT_FLOAT_EQ(4.0, model.x(3));
  ASSERT_FLOAT_EQ(1.0, model.m);
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

  ASSERT_FALSE(sim.configured);
  ASSERT_FALSE(sim.model.configured);

  ASSERT_FLOAT_EQ(0.0, sim.x_init(0));
  ASSERT_FLOAT_EQ(0.0, sim.x_init(1));
  ASSERT_FLOAT_EQ(0.0, sim.x_init(2));
  ASSERT_FLOAT_EQ(0.0, sim.x_init(3));

  ASSERT_FLOAT_EQ(0.0, sim.x_final(0));
  ASSERT_FLOAT_EQ(0.0, sim.x_final(1));
  ASSERT_FLOAT_EQ(0.0, sim.x_final(2));
  ASSERT_FLOAT_EQ(0.0, sim.x_final(3));
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
  ASSERT_TRUE(sim.configured);
  ASSERT_TRUE(sim.model.configured);

  ASSERT_FLOAT_EQ(1.0, sim.x_init(0));
  ASSERT_FLOAT_EQ(2.0, sim.x_init(1));
  ASSERT_FLOAT_EQ(3.0, sim.x_init(2));
  ASSERT_FLOAT_EQ(4.0, sim.x_init(3));

  ASSERT_FLOAT_EQ(11.0, sim.x_final(0));
  ASSERT_FLOAT_EQ(12.0, sim.x_final(1));
  ASSERT_FLOAT_EQ(13.0, sim.x_final(2));
  ASSERT_FLOAT_EQ(14.0, sim.x_final(3));
}

TEST(Simulator, simulate) {
  int retval;
  Simulator sim;
  Vec4 x_init;
  Vec4 x_final;
  double m;
  MatX U;
  Vec2 u;

  // test
  x_init << 1.0, 0.0, 5.0, 0.0;
  x_final << 1.0, 0.0, 5.0, 0.0;
  m = 1.0;

  U.resize(2, 2);
  u << 10.0, 0.0;
  U.block(0, 0, 2, 1) = u;
  u << 10.0, 0.0;
  U.block(0, 1, 2, 1) = u;

  sim.configure(x_init, x_final, m);
  retval = sim.simulate(0.1, 0.2, U);

  ASSERT_EQ(0, retval);
  ASSERT_FLOAT_EQ(0.0, sim.d_az);
  ASSERT_FLOAT_EQ(0.0, sim.d_theta);
  ASSERT_FLOAT_EQ(10.0, sim.az_sum);
}

}  // end of awesomo namespace
