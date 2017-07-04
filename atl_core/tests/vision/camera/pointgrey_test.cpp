#include "atl/atl_test.hpp"
#include "atl/utils/opencv.hpp"
#include "atl/vision/camera/pointgrey.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera/pointgrey_chameleon"


namespace atl {

TEST(PointGreyCamera, constructor) {
  PointGreyCamera camera;

  ASSERT_FALSE(camera.configured);
  ASSERT_FALSE(camera.initialized);

  ASSERT_FALSE(camera.config.loaded);
  ASSERT_EQ(0, camera.modes.size());
  ASSERT_EQ(0, camera.configs.size());

  ASSERT_EQ(NULL, camera.capture);
  ASSERT_FLOAT_EQ(0.0, camera.last_tic);

  ASSERT_EQ(NULL, camera.pointgrey);
}

TEST(PointGreyCamera, configure) {
  int retval;
  PointGreyCamera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);
}

TEST(PointGreyCamera, initialize) {
  int retval;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  retval = camera.initialize();
  ASSERT_EQ(0, retval);
}

TEST(PointGreyCamera, brightness) {
  int retval;
  double brightness;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setBrightness(0.5);
  camera.getBrightness(brightness);

  ASSERT_NEAR(0.5, brightness, 0.2);
}

TEST(PointGreyCamera, frameRate) {
  int retval;
  double fps;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setFrameRate(10);
  camera.getFrameRate(fps);

  ASSERT_FLOAT_EQ(10, fps);
}

TEST(PointGreyCamera, exposure) {
  int retval;
  double exposure;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setExposure(1.0);
  camera.getExposure(exposure);

  ASSERT_FLOAT_EQ(1.0, exposure);
}

TEST(PointGreyCamera, shutter) {
  int retval;
  double shutter;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.setShutter(10);
  camera.getShutter(shutter);

  ASSERT_NEAR(10, shutter, 0.2);
}

TEST(PointGreyCamera, gain) {
  int retval;
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
//   ASSERT_EQ(640, image.cols);
//   ASSERT_EQ(480, image.rows);
//
//   camera.changeMode("320x240");
//   camera.getFrame(image);
//   ASSERT_EQ(320, image.cols);
//   ASSERT_EQ(240, image.rows);
// }

TEST(PointGreyCamera, getFrame) {
  cv::Mat image;
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.getFrame(image);

  // cv::imshow("Image", image);
  // cv::waitKey(100000);

  ASSERT_FALSE(image.empty());
}

TEST(PointGreyCamera, run) {
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  camera.config.imshow = true;
  camera.run();
}

}  // namespace atl
