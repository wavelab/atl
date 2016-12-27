#include <gtest/gtest.h>

#include "awesomo_core/utils/opencv.hpp"
#include "awesomo_core/vision/camera/config.hpp"

#define TEST_CONFIG "tests/configs/camera/640x480.yaml"


TEST(CameraConfig, constructor) {
  awesomo::CameraConfig config;

  ASSERT_FALSE(config.loaded);

  ASSERT_EQ(0, config.index);
  ASSERT_EQ(0, config.image_width);
  ASSERT_EQ(0, config.image_height);

  ASSERT_FLOAT_EQ(0.0, config.exposure_value);
  ASSERT_FLOAT_EQ(0.0, config.gain_value);

  ASSERT_FLOAT_EQ(0.0, config.lambda(0));
  ASSERT_FLOAT_EQ(0.0, config.lambda(1));
  ASSERT_FLOAT_EQ(0.0, config.lambda(2));
  ASSERT_FLOAT_EQ(0.0, config.alpha);

  config.camera_matrix;
  config.rectification_matrix;
  config.distortion_coefficients;
  config.projection_matrix;

  ASSERT_FALSE(config.imshow);
  ASSERT_FALSE(config.snapshot);
}

TEST(CameraConfig, load) {
  // clang-format off
  awesomo::CameraConfig config;
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

  ASSERT_TRUE(config.loaded);

  ASSERT_EQ(0, config.index);
  ASSERT_EQ(640, config.image_width);
  ASSERT_EQ(480, config.image_height);

  ASSERT_FLOAT_EQ(1.0, config.exposure_value);
  ASSERT_FLOAT_EQ(2.0, config.gain_value);
  ASSERT_FLOAT_EQ(1.0, config.lambda(0));
  ASSERT_FLOAT_EQ(2.0, config.lambda(1));
  ASSERT_FLOAT_EQ(3.0, config.lambda(2));
  ASSERT_FLOAT_EQ(4.0, config.alpha);

  // clang-format off
  cv::Mat expected(3, 3, CV_64F, camera_matrix);
  ASSERT_TRUE(awesomo::cvMatIsEqual(expected, config.camera_matrix));

  expected = cv::Mat(1, 5, CV_64F, distortion_coefficients);
  ASSERT_TRUE(awesomo::cvMatIsEqual(expected, config.distortion_coefficients));

  expected = cv::Mat(3, 3, CV_64F, rectification_matrix);
  ASSERT_TRUE(awesomo::cvMatIsEqual(expected, config.rectification_matrix));

  expected = cv::Mat(3, 4, CV_64F, projection_matrix);
  ASSERT_TRUE(awesomo::cvMatIsEqual(expected, config.projection_matrix));
  // clang-format on

  ASSERT_TRUE(config.imshow);
  ASSERT_TRUE(config.snapshot);
}
