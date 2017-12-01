#include "atl/vision/apriltag/michigan.hpp"
#include "atl/atl_test.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

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
#define TEST_ILLUM_INVAR "tests/data/apriltag/illum_invar.png"

TEST(MichiganDetector, constructor) {
  MichiganDetector detector;

  EXPECT_FALSE(detector.configured);

  EXPECT_EQ(nullptr, detector.detector);
  EXPECT_EQ(nullptr, detector.family);

  EXPECT_EQ(0, detector.tag_configs.size());
  EXPECT_EQ("", detector.camera_mode);
  EXPECT_EQ(0, detector.camera_modes.size());
  EXPECT_EQ(0, detector.camera_configs.size());
  EXPECT_FALSE(detector.imshow);
}

TEST(MichiganDetector, configure) {
  MichiganDetector detector;

  detector.configure(TEST_CONFIG);

  EXPECT_TRUE(detector.configured);

  EXPECT_FALSE(detector.detector == nullptr);

  EXPECT_EQ(2, detector.tag_configs.size());
  EXPECT_EQ(detector.camera_modes[0], detector.camera_mode);
  EXPECT_EQ(3, detector.camera_modes.size());
  EXPECT_EQ(3, detector.camera_configs.size());
  EXPECT_FALSE(detector.imshow);
}

TEST(MichiganDetector, extractTags) {
  MichiganDetector detector;
  detector.configure(TEST_CONFIG);

  // CENTER
  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  std::vector<TagPose> tags;
  int retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;

  // cv::imshow("image", image);
  // cv::waitKey(10000);

  EXPECT_EQ(0, retval);
  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.2, tags[0].position(2), 0.15);
  tags.clear();

  // TOP
  image = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;

  EXPECT_EQ(0, retval);
  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(-0.5, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.4, tags[0].position(2), 0.15);
  tags.clear();

  // RIGHT
  image = cv::imread(TEST_IMAGE_RIGHT, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;

  EXPECT_EQ(0, retval);
  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.5, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.30, tags[0].position(2), 0.15);
  tags.clear();
}

TEST(MichiganDetector, sandbox) {
  MichiganDetector detector;
  std::vector<TagPose> tags;

  detector.configure(TEST_CONFIG);

  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  detector.extractTags(image, tags);
  tags[0].print();
  tags.clear();

  detector.extractTags(image, tags);
  tags[0].print();
}

} // namespace atl
