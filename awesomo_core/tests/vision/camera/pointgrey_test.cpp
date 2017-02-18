#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/utils/opencv.hpp"
#include "awesomo_core/vision/camera/pointgrey.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera/pointgrey_chameleon"


namespace awesomo {

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
  ASSERT_FLOAT_EQ(0.5, camera.shutter_speed);
}

TEST(PointGreyCamera, configure) {
  int retval;
  PointGreyCamera camera;

  retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);
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

  // ASSERT_FALSE(image.empty());
}

TEST(PointGreyCamera, run) {
  PointGreyCamera camera;

  camera.configure(TEST_CONFIG_PATH);
  camera.initialize();
  // camera.config.imshow = true;
  camera.run();
}

}  // end of awesomo namespace
