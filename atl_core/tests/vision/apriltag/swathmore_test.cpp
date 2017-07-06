#include "atl/atl_test.hpp"
#include "atl/vision/camera/camera.hpp"
#include "atl/vision/apriltag/swathmore.hpp"

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

namespace atl {

TEST(SwathmoreDetector, constructor) {
  SwathmoreDetector detector;

  EXPECT_FALSE(detector.configured);

  EXPECT_EQ(NULL, detector.detector);

  EXPECT_EQ(0, detector.tag_configs.size());
  EXPECT_EQ("", detector.camera_mode);
  EXPECT_EQ(0, detector.camera_modes.size());
  EXPECT_EQ(0, detector.camera_configs.size());
  EXPECT_FALSE(detector.imshow);
}

TEST(SwathmoreDetector, configure) {
  SwathmoreDetector detector;

  detector.configure(TEST_CONFIG);
  EXPECT_TRUE(detector.configured);

  EXPECT_FALSE(detector.detector == NULL);

  EXPECT_EQ(2, detector.tag_configs.size());
  EXPECT_EQ(detector.camera_modes[0], detector.camera_mode);
  EXPECT_EQ(3, detector.camera_modes.size());
  EXPECT_EQ(3, detector.camera_configs.size());
  EXPECT_TRUE(detector.imshow);
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

  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.2, tags[0].position(2), 0.15);

  // TOP
  image = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  tags = detector.extractTags(image);
  tags[0].print();

  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(-0.5, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.4, tags[0].position(2), 0.15);

  // RIGHT
  image = cv::imread(TEST_IMAGE_RIGHT, CV_LOAD_IMAGE_COLOR);
  tags = detector.extractTags(image);
  tags[0].print();

  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.5, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.30, tags[0].position(2), 0.15);
}

}  // namespace atl
