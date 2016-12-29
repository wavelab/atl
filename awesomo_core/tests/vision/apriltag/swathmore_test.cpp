#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/vision/camera/camera.hpp"
#include "awesomo_core/vision/apriltag/swathmore.hpp"

#define TEST_CONFIG "tests/configs/pointgrey"
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

static std::vector<TagPose> process_image(SwathmoreDetector &detector,
                                          std::string image_file) {
  int timeout;
  cv::Mat image;
  cv::Mat camera_matrix;
  std::vector<TagPose> tags;

  // setup
  timeout = 0;
  image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  // tags = detector.processImage(camera_matrix, image, timeout);

  return tags;
}

TEST(SwathmoreDetector, constructor) {
  SwathmoreDetector detector;

  ASSERT_FALSE(detector.configured);

  ASSERT_EQ(NULL, detector.detector);

  ASSERT_EQ(0.0, detector.tag_sizes.size());
  ASSERT_FALSE(detector.imshow);
}

TEST(SwathmoreDetector, configure) {
  SwathmoreDetector detector;

  detector.configure(TEST_CONFIG);
  ASSERT_TRUE(detector.configured);

  ASSERT_FALSE(detector.detector == NULL);

  // ASSERT_EQ(0.0, detector.tag_sizes.size());
  // ASSERT_FALSE(detector.imshow);
}

}  // end of awesomo namespace
