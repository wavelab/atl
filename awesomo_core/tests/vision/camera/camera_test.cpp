#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/utils/opencv.hpp"
#include "awesomo_core/vision/camera/camera.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera/webcam"


namespace awesomo {

TEST(Camera, constructor) {
  Camera camera;

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
  Camera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);
}

TEST(Camera, changeMode) {
  cv::Mat image;
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();

  camera.getFrame(image);
  ASSERT_EQ(640, image.cols);
  ASSERT_EQ(480, image.rows);

  camera.changeMode("320x240");
  camera.getFrame(image);
  ASSERT_EQ(320, image.cols);
  ASSERT_EQ(240, image.rows);
}

TEST(Camera, getFrame) {
  cv::Mat image;
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.getFrame(image);

  ASSERT_FALSE(image.empty());
}

// TEST(Camera, run) {
//   Camera camera;
//
//   camera.configure(TEST_CONFIG_PATH);
//   camera.initialize();
//   camera.run();
// }

}  // end of awesomo namespace
