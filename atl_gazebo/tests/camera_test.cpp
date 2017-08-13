#include <gtest/gtest.h>

#include "atl/gazebo/clients/camera_gclient.hpp"
#include "atl/gazebo/gazebo_test.hpp"
#include "test_settings.hpp"

#define TEST_IMAGE "tests/data/camera_test.png"
#define TEST_WORLD "tests/worlds/camera_test.world"

namespace atl {
namespace gaz {

class CameraTest : public ::testing::Test {
protected:
  CameraGClient camera_client;
  WorldGClient world_client;
  GazeboTest gazebo_test;
  std::string test_world;

  CameraTest() { this->test_world = "atl_gazebo/" + std::string(TEST_WORLD); }

  virtual ~CameraTest() { this->gazebo_test.teardown(); }

  virtual void SetUp() {
    // world client
    if (this->world_client.configure() == -1) {
      std::cout << "Failed to connect to Gazebo Server!" << std::endl;
      exit(-1);
    }
    this->world_client.clearWorld();
    this->world_client.resetWorld();
    this->world_client.loadWorld(this->test_world);

    // camera client
    if (this->camera_client.configure() == -1) {
      std::cout << "CameraClient Failed to connect to Gazebo!" << std::endl;
      exit(-1);
    }
  }

  virtual void TearDown() {
    this->world_client.clearWorld();
    this->world_client.resetWorld();
  }
};

TEST_F(CameraTest, test) {
  cv::Mat image;

  // test and assert
  sleep(5);
  image = this->camera_client.image.clone();

  // debug visually
  // cv::imshow("got image", image);
  // cv::waitKey(100000);

  EXPECT_FALSE(image.empty());
}

} // namespace gaz
} // namespace atl

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
