#include "atl/vision/camera/config.hpp"
#include "atl/atl_test.hpp"
#include "atl/utils/opencv.hpp"

#define TEST_CONFIG "tests/configs/camera/webcam/640x480.yaml"

TEST(CameraConfig, constructor) {
  atl::CameraConfig config;

  EXPECT_FALSE(config.loaded);

  EXPECT_EQ(0, config.index);
  EXPECT_EQ(0, config.image_width);
  EXPECT_EQ(0, config.image_height);

  EXPECT_FLOAT_EQ(0.0, config.exposure_value);
  EXPECT_FLOAT_EQ(0.0, config.gain_value);

  EXPECT_FLOAT_EQ(0.0, config.lambda(0));
  EXPECT_FLOAT_EQ(0.0, config.lambda(1));
  EXPECT_FLOAT_EQ(0.0, config.lambda(2));
  EXPECT_FLOAT_EQ(0.0, config.alpha);

  // config.camera_matrix;
  // config.rectification_matrix;
  // config.distortion_coefficients;
  // config.projection_matrix;

  EXPECT_FALSE(config.imshow);
  EXPECT_FALSE(config.snapshot);
}

TEST(CameraConfig, load) {
  // clang-format off
  atl::CameraConfig config;
  double camera_matrix[] = {1.0, 2.0, 3.0,
                            4.0, 5.0, 6.0,
                            7.0, 8.0, 9.0};
  double distortion_coefficients[] = {1.0, 2.0, 3.0, 4.0, 5.0};
  double rectification_matrix[] = {1.0, 2.0, 3.0,
                                   4.0, 5.0, 6.0,
                                   7.0, 8.0, 9.0};
  double projection_matrix[] = {1.0, 2.0, 3.0, 4.0,
                                5.0, 6.0, 7.0, 8.0,
                                9.0, 10.0, 11.0, 12.0};
  // clang-format on

  // test and assert
  config.load(TEST_CONFIG);

  EXPECT_TRUE(config.loaded);

  EXPECT_EQ(0, config.index);
  EXPECT_EQ(640, config.image_width);
  EXPECT_EQ(480, config.image_height);

  EXPECT_FLOAT_EQ(1.0, config.exposure_value);
  EXPECT_FLOAT_EQ(2.0, config.gain_value);
  EXPECT_FLOAT_EQ(1.0, config.lambda(0));
  EXPECT_FLOAT_EQ(2.0, config.lambda(1));
  EXPECT_FLOAT_EQ(3.0, config.lambda(2));
  EXPECT_FLOAT_EQ(4.0, config.alpha);

  // clang-format off
  cv::Mat expected(3, 3, CV_64F, camera_matrix);
  EXPECT_TRUE(atl::cvMatIsEqual(expected, config.camera_matrix));

  expected = cv::Mat(1, 5, CV_64F, distortion_coefficients);
  EXPECT_TRUE(atl::cvMatIsEqual(expected, config.distortion_coefficients));

  expected = cv::Mat(3, 3, CV_64F, rectification_matrix);
  EXPECT_TRUE(atl::cvMatIsEqual(expected, config.rectification_matrix));

  expected = cv::Mat(3, 4, CV_64F, projection_matrix);
  EXPECT_TRUE(atl::cvMatIsEqual(expected, config.projection_matrix));
  // clang-format on

  EXPECT_TRUE(config.imshow);
  EXPECT_TRUE(config.snapshot);
}
