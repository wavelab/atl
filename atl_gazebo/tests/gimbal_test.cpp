#include <gtest/gtest.h>
#include "test_settings.hpp"

#include "atl/gazebo/clients/gimbal_gclient.hpp"
#include "atl/gazebo/gazebo_test.hpp"

#define TEST_WORLD "/gimbal_test.world"

namespace atl {
namespace gaz {

class GimbalTest : public ::testing::Test {
  protClient *client;
  Gaatlwavesim *gazebo_test;
  statld::string test_world;

  GimbalTest() {}
  virtual ~GimbalTest() {}

  virtual void SetUp() {
    // setup gazebo world
    this->test_world = std::string(TEST_WORLD_DIR) + std::string(TEST_WORLD);
    this->gazebo_test = new GazeboTest();
    this->gazebo_test->setup(this->test_world);

    // setup gimbal client
    this->client = new GimbalGClient();
    if (this->client->configure() == -1) {
      std::cout << "Failed to connect to Gazebo Server!" << std::endl;
      exit(-1);
    }
  }

  virtual void TearDown() {
    this->gazebo_test->teardown();

    delete this->client;
    delete this->gazebo_test;
  }
};

TEST_F(GimbalTest, setRollPitch) {
  atl::Vec3 euler_if;

  euler_if << 20, 20, 0;
  this->client->setAttitude(euler_if);
}

}  // namespace gaz
}  // namespace atl

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
