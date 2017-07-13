#include "atl/vision/apriltag/mit.hpp"
#include "atl/atl_test.hpp"
#include "atl/vision/camera/camera.hpp"

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

namespace atl {

TEST(MITDetector, constructor) {
  MITDetector detector;

  EXPECT_FALSE(detector.configured);

  EXPECT_EQ(NULL, detector.detector);

  EXPECT_EQ(0, detector.tag_configs.size());
  EXPECT_EQ("", detector.camera_mode);
  EXPECT_EQ(0, detector.camera_modes.size());
  EXPECT_EQ(0, detector.camera_configs.size());
  EXPECT_FALSE(detector.imshow);
}

TEST(MITDetector, configure) {
  MITDetector detector;

  detector.configure(TEST_CONFIG);
  EXPECT_TRUE(detector.configured);

  EXPECT_FALSE(detector.detector == NULL);

  EXPECT_EQ(2, detector.tag_configs.size());
  EXPECT_EQ(detector.camera_modes[0], detector.camera_mode);
  EXPECT_EQ(3, detector.camera_modes.size());
  EXPECT_EQ(3, detector.camera_configs.size());
  EXPECT_FALSE(detector.imshow);
}

TEST(MITDetector, illuminationInvarientTransform) {
  cv::Mat image;
  MITDetector detector;
  std::vector<AprilTags::TagDetection> tags;

  // setup
  detector.configure(TEST_CONFIG);
  image = cv::imread(TEST_ILLUM_INVAR, CV_LOAD_IMAGE_COLOR);

  // test and assert
  detector.illuminationInvariantTransform(image);
  // cv::imshow("image", image);
  // cv::waitKey(1000000);

  tags = detector.detector->extractTags(image);
  EXPECT_EQ(2, tags.size());
}

TEST(MITDetector, extractTags) {
  // setup
  MITDetector detector;
  detector.configure(TEST_CONFIG);

  // CENTER
  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  std::vector<TagPose> tags;
  int retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

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
  // tags[0].print();

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
  // tags[0].print();

  EXPECT_EQ(0, retval);
  EXPECT_EQ(1, tags.size());
  ASSERT_NEAR(0.5, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.30, tags[0].position(2), 0.15);
  tags.clear();
}

TEST(MITDetector, changeMode) {
  MITDetector detector;
  cv::Mat image1, image2, image3;

  // setup
  detector.configure(TEST_CONFIG);

  image1 = cv::Mat(480, 640, CV_64F, double(0));
  detector.changeMode(image1);
  EXPECT_EQ("640x480", detector.camera_mode);

  image2 = cv::Mat(240, 320, CV_64F, double(0));
  detector.changeMode(image2);
  EXPECT_EQ("320x240", detector.camera_mode);

  image3 = cv::Mat(120, 160, CV_64F, double(0));
  detector.changeMode(image3);
  EXPECT_EQ("160x120", detector.camera_mode);
}

TEST(MITDetector, maskImage) {
  // setup
  MITDetector detector;
  detector.configure(TEST_CONFIG);

  // CENTER
  cv::Mat image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  std::vector<TagPose> tags;
  detector.extractTags(image, tags);
  detector.maskImage(tags[0], image);

  // cv::imshow("test", image);
  // cv::waitKey(100000);
}

}  // namespace atl
