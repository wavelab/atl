#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/mit.hpp"

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

TEST(MITDetector, constructor) {
  MITDetector detector;

  ASSERT_FALSE(detector.configured);

  ASSERT_EQ(NULL, detector.detector);

  ASSERT_EQ(0, detector.tag_configs.size());
  ASSERT_EQ("", detector.camera_mode);
  ASSERT_EQ(0, detector.camera_modes.size());
  ASSERT_EQ(0, detector.camera_configs.size());
  ASSERT_FALSE(detector.imshow);
}

TEST(MITDetector, configure) {
  MITDetector detector;

  detector.configure(TEST_CONFIG);
  ASSERT_TRUE(detector.configured);

  ASSERT_FALSE(detector.detector == NULL);

  ASSERT_EQ(2, detector.tag_configs.size());
  ASSERT_EQ(detector.camera_modes[0], detector.camera_mode);
  ASSERT_EQ(3, detector.camera_modes.size());
  ASSERT_EQ(3, detector.camera_configs.size());
  ASSERT_FALSE(detector.imshow);
}

TEST(MITDetector, extractTags) {
  int retval;
  cv::Mat image;
  std::vector<TagPose> tags;
  MITDetector detector;

  // setup
  detector.configure(TEST_CONFIG);

  // CENTER
  image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

  ASSERT_EQ(0, retval);
  ASSERT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(0.0, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.2, tags[0].position(2), 0.15);
  tags.clear();

  // TOP
  image = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

  ASSERT_EQ(0, retval);
  ASSERT_EQ(1, tags.size());
  ASSERT_NEAR(0.0, tags[0].position(0), 0.15);
  ASSERT_NEAR(-0.5, tags[0].position(1), 0.15);
  ASSERT_NEAR(2.4, tags[0].position(2), 0.15);
  tags.clear();

  // RIGHT
  image = cv::imread(TEST_IMAGE_RIGHT, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.prev_tag.detected = false;
  // tags[0].print();

  ASSERT_EQ(0, retval);
  ASSERT_EQ(1, tags.size());
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
  ASSERT_EQ("640x480", detector.camera_mode);

  image2 = cv::Mat(240, 320, CV_64F, double(0));
  detector.changeMode(image2);
  ASSERT_EQ("320x240", detector.camera_mode);

  image3 = cv::Mat(120, 160, CV_64F, double(0));
  detector.changeMode(image3);
  ASSERT_EQ("160x120", detector.camera_mode);
}

TEST(MITDetector, maskImage) {
  int retval;
  MITDetector detector;
  cv::Mat image, masked;
  std::vector<TagPose> tags;

  // setup
  detector.configure(TEST_CONFIG);

  // CENTER
  image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
  retval = detector.extractTags(image, tags);
  detector.maskImage(tags[0], image, masked);

  // cv::imshow("test", masked);
  // cv::waitKey(100000);
}

}  // end of awesomo namespace
