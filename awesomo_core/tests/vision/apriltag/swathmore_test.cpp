#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/swathmore.hpp"

#define TEST_CONFIG "tests/configs/apriltag/config.yaml"
#define TEST_IMAGE_CENTER "tests/data/center.png"
#define TEST_IMAGE_TOP "tests/data/top.png"
#define TEST_IMAGE_BOTTOM "tests/data/bottom.png"
#define TEST_IMAGE_LEFT "tests/data/left.png"
#define TEST_IMAGE_RIGHT "tests/data/right.png"
#define TEST_IMAGE_TOP_LEFT "tests/data/top_left.png"
#define TEST_IMAGE_BOTTOM_LEFT "tests/data/bottom_left.png"
#define TEST_IMAGE_TOP_RIGHT "tests/data/top_right.png"
#define TEST_IMAGE_BOTTOM_RIGHT "tests/data/bottom_right.png"

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
  ASSERT_EQ(1, detector.camera_modes.size());
  ASSERT_EQ(1, detector.camera_configs.size());
  ASSERT_TRUE(detector.imshow);
}

// TEST(SwathmoreDetector, extractTags) {
//   cv::Mat image;
//   std::vector<TagPose> tags;
//   SwathmoreDetector detector;
//
//   detector.configure(TEST_CONFIG);
//   image = cv::imread(TEST_IMAGE_CENTER, CV_LOAD_IMAGE_COLOR);
//   tags = detector.extractTags(image);
// }

}  // end of awesomo namespace
