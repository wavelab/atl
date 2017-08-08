#include "atl/vision/camera/dc1394.hpp"
#include "atl/atl_test.hpp"
#include "atl/utils/opencv.hpp"

#define TEST_CONFIG_PATH "tests/configs/camera/pointgrey_chameleon"

namespace atl {

TEST(DC1394Camera, constructor) {
  DC1394Camera camera;

  EXPECT_FALSE(camera.configured);
  EXPECT_FALSE(camera.initialized);

  EXPECT_FALSE(camera.config.loaded);
  EXPECT_EQ(0, camera.modes.size());
  EXPECT_EQ(0, camera.configs.size());

  EXPECT_EQ(NULL, camera.capture);
  EXPECT_FLOAT_EQ(0.0, camera.last_tic);
}


TEST(DC1394Camera, configure) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);
}

TEST(DC1394Camera, framerate) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  EXPECT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  double fps_expected = 60.0;
  retval = camera.setFrameRate(fps_expected);
  EXPECT_EQ(0, retval);

  double fps = 0;
  retval = camera.getFrameRate(fps);
  EXPECT_EQ(fps_expected, fps);
  EXPECT_EQ(0, retval);
}

// TEST(DC1394Camera, run) {
//   DC1394Camera camera;
//
//   int retval = camera.configure(TEST_CONFIG_PATH);
//   ASSERT_EQ(0, retval);
//
//   retval = camera.initialize();
//   ASSERT_EQ(0, retval);
//
//   camera.run();
// }

}  // namespace atl
