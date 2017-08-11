#include <gtest/gtest.h>

#include "atl/gazebo/clients/world_gclient.hpp"
#include "atl/gazebo/gazebo_test.hpp"
#include "test_settings.hpp"

#define TEST_WORLD "tests/worlds/world_test.world"

namespace atl {
namespace gaz {

class WorldTest : public ::testing::Test {
protected:
  WorldGClient client;
  std::string test_world;

  WorldTest() {
    // setup gazebo world
    this->test_world = "atl_gazebo/" + std::string(TEST_WORLD);

    // setup world client
    if (this->client.configure() == -1) {
      std::cout << "Failed to connect to Gazebo Server!" << std::endl;
      exit(-1);
    }
  }

  virtual ~WorldTest() { this->client.clearWorld(); }

  virtual void SetUp() {
    this->client.clearWorld();
    this->client.resetWorld();
    this->client.loadWorld(this->test_world);
  }

  virtual void TearDown() {
    this->client.clearWorld();
    this->client.resetWorld();
  }
};

TEST_F(WorldTest, PauseAndUnPauseWorld) {
  // test pause
  this->client.pauseWorld();
  sleep(1);
  EXPECT_TRUE(this->client.time.paused);

  // test unpause
  this->client.unPauseWorld();
  sleep(1);
  EXPECT_FALSE(this->client.time.paused);
}

TEST_F(WorldTest, Reset) {
  // test reset
  sleep(3);
  this->client.resetWorld();
  sleep(1);
  EXPECT_TRUE(this->client.time.sim_time.sec() < 2);
}

TEST_F(WorldTest, LoadAndRemoveModel) {
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;

  pos << 0.0, 0.0, 2.0;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  // test load model
  this->client.loadModel("table", pos, quat);
}

TEST_F(WorldTest, ClearAndLoadWorld) {
  this->client.clearWorld();
  this->client.loadWorld("worlds/quadrotor_test.world");
}

} // namespace gaz
} // namespace atl

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
