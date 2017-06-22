#include <gtest/gtest.h>
#include "test_settings.hpp"

#include "wavesim_gazebo/gazebo_test.hpp"
#include "wavesim_gazebo/clients/gimbal_gclient.hpp"

#define TEST_WORLD "/gimbal_test.world"

namespace wavesim {
namespace gaz {

class GimbalTest : public ::testing::Test {
  protClient *client;
  Gawavesimwavesim *gazebo_test;
  stwavesimd::string test_world;

  GimbalTest(void) {}
  virtual ~GimbalTest(void) {}

  virtual void SetUp(void) {
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

  virtual void TearDown(void) {
    this->gazebo_test->teardown();

    delete this->client;
    delete this->gazebo_test;
  }
};

TEST_F(GimbalTest, setRollPitch) {
  wavesim::Vec3 euler_if;

  euler_if << 20, 20, 0;
  this->client->setAttitude(euler_if);
}

}  // end of gaz namespace
}  // end of wavesim namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
