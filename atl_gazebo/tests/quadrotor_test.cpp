#include <gtest/gtest.h>

#include "atl/gazebo/clients/quadrotor_gclient.hpp"
#include "atl/gazebo/gazebo_test.hpp"
#include "test_settings.hpp"

#define TEST_WORLD "/quadrotor_test.world"

namespace atl {
namespace gaz {

class QuadrotorTest : public ::testing::Test {
protected:
  QuadrotorGClient *client;
  GazeboTest *gazebo_test;
  std::string test_world;

  QuadrotorTest(void) {}
  virtual ~QuadrotorTest(void) {}

  virtual void SetUp(void) {
    // setup gazebo world
    this->test_world = std::string(TEST_WORLD_DIR) + std::string(TEST_WORLD);
    this->gazebo_test = new GazeboTest();
    this->gazebo_test->setup(this->test_world);

    // setup quadrotor client
    this->client = new QuadrotorGClient();
    if (this->client->configure() == -1) {
      std::cout << "Failed to connect to Gazebo Server!" << std::endl;
      exit(-1);
    }
  }

  virtual void TearDown(void) {
    this->gazebo_test->teardown();

    delete this->client;
    delete this->gazebo_test;
  }
};

TEST_F(QuadrotorTest, setPosition) {
  // setpoint 1
  this->client->setPosition(0.0, 0.0, 3.0);
  sleep(1);
  EXPECT_FLOAT_EQ(0.0, this->client->ground_truth(0));
  EXPECT_FLOAT_EQ(0.0, this->client->ground_truth(1));
  EXPECT_FLOAT_EQ(3.0, this->client->ground_truth(2));

  // setpoint 2
  this->client->setPosition(5.0, 5.0, 3.0);
  sleep(7);
  EXPECT_TRUE(5.8 > this->client->ground_truth(0));
  EXPECT_TRUE(4.8 < this->client->ground_truth(0));
  EXPECT_TRUE(5.8 > this->client->ground_truth(1));
  EXPECT_TRUE(4.8 < this->client->ground_truth(1));
  EXPECT_TRUE(3.8 > this->client->ground_truth(2));
  EXPECT_TRUE(2.8 < this->client->ground_truth(2));

  // setpoint 3
  this->client->setPosition(0.0, 0.0, 3.0);
  sleep(5);
  EXPECT_TRUE(0.8 > this->client->ground_truth(0));
  EXPECT_TRUE(-0.8 < this->client->ground_truth(0));
  EXPECT_TRUE(0.8 > this->client->ground_truth(1));
  EXPECT_TRUE(-0.8 < this->client->ground_truth(1));
  EXPECT_TRUE(3.8 > this->client->ground_truth(2));
  EXPECT_TRUE(2.8 < this->client->ground_truth(2));
}

// TEST_F(QuadrotorTest, setVelocity) {
//   // velocity 1
//   this->client->setVelocity(1.0, 0.0, 0.0);
//   sleep(10);
// }

}  // namespace gaz
}  // namespace atl

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
