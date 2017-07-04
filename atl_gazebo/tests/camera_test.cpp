#include <gtest/gtest.h>

#include "test_settings.hpp"
#include "atl_gazebo/gazebo_test.hpp"
#include "atl_gazebo/clients/camera_gclient.hpp"

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

  CameraTest(void) {
    this->test_world = "atl_gazebo/" + std::string(TEST_WORLD);
  }

  virtual ~CameraTest(void) {
    this->gazebo_test.teardown();
  }

  virtual void SetUp(void) {
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

  virtual void TearDown(void) {
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

  ASSERT_FALSE(image.empty());
}

}  // end of gaz namespace
}  // end of atl namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
