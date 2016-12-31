#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/utils/opencv.hpp"
#include "awesomo_core/vision/camera/camera.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera"

TEST(Camera, constructor) {
  awesomo::Camera camera;

  ASSERT_FALSE(camera.configured);
  ASSERT_FALSE(camera.initialized);

  ASSERT_FALSE(camera.config.loaded);
  ASSERT_EQ(0, camera.modes.size());
  ASSERT_EQ(0, camera.configs.size());

  ASSERT_EQ(NULL, camera.capture);
  ASSERT_FLOAT_EQ(0.0, camera.last_tic);
}

TEST(Camera, configure) {
  int retval;
  awesomo::Camera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);
}

TEST(Camera, getFrame) {
  cv::Mat image;
  awesomo::Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.getFrame(image);

  ASSERT_FALSE(image.empty());
}

// TEST(Camera, run) {
//   awesomo::Camera camera;
//
//   camera.configure(TEST_CONFIG_PATH);
//   camera.initialize();
//   camera.run();
// }
