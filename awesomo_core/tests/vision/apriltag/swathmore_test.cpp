#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/swathmore.hpp"

#define TEST_CONFIG "tests/configs/apriltag/config.yaml"
#define TEST_IMAGE_CENTER "tests/data/apriltag/center.png"
#define TEST_IMAGE_TOP "tests/data/apriltag/top.png"
#define TEST_IMAGE_BOTTOM "tests/data/apriltag/bottom.png"
#define TEST_IMAGE_LEFT "tests/data/apriltag/left.png"
#define TEST_IMAGE_RIGHT "tests/data/apriltag/right.png"
#define TEST_IMAGE_TOP_LEFT "tests/data/apriltag/top_left.png"
#define TEST_IMAGE_BOTTOM_LEFT "tests/data/apriltag/bottom_left.png"
#define TEST_IMAGE_TOP_RIGHT "tests/data/apriltag/top_right.png"
#define TEST_IMAGE_BOTTOM_RIGHT "tests/data/apriltag/bottom_right.png"

namespace awesomo {

TEST(SwathmoreDetector, constructor) {
  SwathmoreDetector detector;

  ASSERT_FALSE(detector.configured);

  ASSERT_EQ(NULL, detector.detector);

  ASSERT_EQ(0, detector.tag_configs.size());
  ASSERT_EQ("", detector.camera_mode);
  ASSERT_EQ(0, detector.camera_modes.size());
  ASSERT_EQ(0, detector.camera_configs.size());
  ASSERT_FALSE(detector.imshow);
}

TEST(SwathmoreDetector, configure) {
  SwathmoreDetector detector;

  detector.configure(TEST_CONFIG);
  ASSERT_TRUE(detector.configured);

  ASSERT_FALSE(detector.detector == NULL);

  ASSERT_EQ(2, detector.tag_configs.size());
  ASSERT_EQ(detector.camera_modes[0], detector.camera_mode);
  ASSERT_EQ(3, detector.camera_modes.size());
  ASSERT_EQ(3, detector.camera_configs.size());
  ASSERT_TRUE(detector.imshow);
}

TEST(SwathmoreDetector, extractTags) {
  cv::Mat image;
  std::vector<TagPose> tags;
  SwathmoreDetector detector;

  // setup
  detector.configure(TEST_CONFIG);

  // CENTER
  image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  tags = detector.extractTags(image);
  tags[0].print();

  ASSERT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.2, tags[0].position(2), 0.15);

  // TOP
  image = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  tags = detector.extractTags(image);
  tags[0].print();

  ASSERT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(-0.5, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.4, tags[0].position(2), 0.15);

  // RIGHT
  image = cv::imread(TEST_IMAGE_RIGHT, CV_LOAD_IMAGE_COLOR);
  tags = detector.extractTags(image);
  tags[0].print();

  ASSERT_EQ(1, tags.size());
  ASSERT_NEAR(0.5, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.30, tags[0].position(2), 0.15);
}

}  // end of awesomo namespace
