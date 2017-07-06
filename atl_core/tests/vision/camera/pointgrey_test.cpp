#include "atl/atl_test.hpp"
#include "atl/utils/opencv.hpp"
#include "atl/vision/camera/pointgrey.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera/pointgrey_chameleon"


namespace atl {

TEST(PointGreyCamera, constructor) {
  PointGreyCamera camera;

  EXPECT_FALSE(camera.configured);
  EXPECT_FALSE(camera.initialized);

  EXPECT_FALSE(camera.config.loaded);
  EXPECT_EQ(0, camera.modes.size());
  EXPECT_EQ(0, camera.configs.size());

  EXPECT_EQ(NULL, camera.capture);
  EXPECT_FLOAT_EQ(0.0, camera.last_tic);

  EXPECT_EQ(NULL, camera.pointgrey);
}

TEST(PointGreyCamera, configure) {
  int retval;
  PointGreyCamera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, retval);
}

TEST(PointGreyCamera, initialize) {
  int retval;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  retval = camera.initialize();
  EXPECT_EQ(0, retval);
}

TEST(PointGreyCamera, brightness) {
  double brightness;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setBrightness(0.5);
  camera.getBrightness(brightness);

  ASSERT_NEAR(0.5, brightness, 0.2);
}

TEST(PointGreyCamera, frameRate) {
  double fps;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setFrameRate(10);
  camera.getFrameRate(fps);

  EXPECT_FLOAT_EQ(10, fps);
}

TEST(PointGreyCamera, exposure) {
  double exposure;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setExposure(1.0);
  camera.getExposure(exposure);

  EXPECT_FLOAT_EQ(1.0, exposure);
}

TEST(PointGreyCamera, shutter) {
  double shutter;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setShutter(10);
  camera.getShutter(shutter);

  ASSERT_NEAR(10, shutter, 0.2);
}

TEST(PointGreyCamera, gain) {
  double gain;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setGain(5.0);
  camera.getGain(gain);

  ASSERT_NEAR(5.0, gain, 0.2);
}

TEST(PointGreyCamera, printFormat7Capabilities) {
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.printFormat7Capabilities();
}

// TEST(PointGreyCamera, changeMode) {
//   cv::Mat image;
//   PointGreyCamera camera;
//
//   camera.configure(TEST_CONFIG_PATH);
//   camera.initialize();
//
//   camera.getFrame(image);
//   EXPECT_EQ(640, image.cols);
//   EXPECT_EQ(480, image.rows);
//
//   camera.changeMode("320x240");
//   camera.getFrame(image);
//   EXPECT_EQ(320, image.cols);
//   EXPECT_EQ(240, image.rows);
// }

TEST(PointGreyCamera, getFrame) {
  cv::Mat image;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.getFrame(image);

  // cv::imshow("Image", image);
  // cv::waitKey(100000);

  EXPECT_FALSE(image.empty());
}

TEST(PointGreyCamera, run) {
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.config.imshow = true;
  camera.run();
}

}  // namespace atl
