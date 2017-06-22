#ifndef WAVESIM_GAZEBO_TEST_HPP
#define WAVESIM_GAZEBO_TEST_HPP

#include "wavesim_gazebo/clients/world_gclient.hpp"


namespace wavesim {
namespace gaz {

class GazeboTest {
public:
  WorldGClient world_client;

  GazeboTest(void) {
    if (this->world_client.configure() == -1) {
      std::cout << "Failed to connect to Gazebo Server!" << std::endl;
      exit(-1);
    }
  }

  void setup(std::string test_world) {
    this->world_client.clearWorld();
    this->world_client.resetWorld();
    this->world_client.loadWorld(test_world);
  }

  void teardown(void) {
    this->world_client.clearWorld();
  }
};

}  // end of gaz namespace
}  // end of wavesim namespace
#endif
