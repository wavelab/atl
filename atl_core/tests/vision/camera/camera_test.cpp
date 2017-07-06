#include "atl/atl_test.hpp"
#include "atl/utils/opencv.hpp"
#include "atl/vision/camera/camera.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera/webcam"


namespace atl {

TEST(Camera, constructor) {
  Camera camera;

  EXPECT_FALSE(camera.configured);
  EXPECT_FALSE(camera.initialized);

  EXPECT_FALSE(camera.config.loaded);
  EXPECT_EQ(0, camera.modes.size());
  EXPECT_EQ(0, camera.configs.size());

  EXPECT_EQ(NULL, camera.capture);
  EXPECT_FLOAT_EQ(0.0, camera.last_tic);
}

TEST(Camera, configure) {
  int retval;
  Camera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, retval);
}

TEST(Camera, changeMode) {
  cv::Mat image;
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();

  camera.getFrame(image);
  EXPECT_EQ(640, image.cols);
  EXPECT_EQ(480, image.rows);

  camera.changeMode("320x240");
  camera.getFrame(image);
  EXPECT_EQ(320, image.cols);
  EXPECT_EQ(240, image.rows);
}

TEST(Camera, getFrame) {
  cv::Mat image;
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.getFrame(image);

  EXPECT_FALSE(image.empty());
}

TEST(Camera, run) {
  Camera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.run();
}

}  // namespace atl
