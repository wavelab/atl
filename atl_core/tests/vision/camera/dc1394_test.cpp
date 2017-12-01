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

TEST(DC1394Camera, triggerMode) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  int trigger_mode_expected = 0;
  retval = camera.setTriggerMode(trigger_mode_expected);
  EXPECT_EQ(0, retval);

  int trigger_mode = -1;
  retval = camera.getTriggerMode(trigger_mode);
  EXPECT_EQ(trigger_mode_expected, trigger_mode);
  EXPECT_EQ(0, retval);

  // revert back to internal trigger
  retval = camera.setTriggerMode(3);
  EXPECT_EQ(0, retval);
}

TEST(DC1394Camera, triggerSource) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  int trigger_source_expected = 4;
  retval = camera.setTriggerSource(trigger_source_expected);
  EXPECT_EQ(0, retval);

  int trigger_source = -1;
  retval = camera.getTriggerSource(trigger_source);
  EXPECT_EQ(trigger_source_expected, trigger_source);
  EXPECT_EQ(0, retval);

  // revert back to trigger source 0
  retval = camera.setTriggerSource(0);
  EXPECT_EQ(0, retval);
}

TEST(DC1394Camera, externalTrigger) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  bool activate_expected = true;
  retval = camera.setExternalTriggering(activate_expected);
  EXPECT_EQ(0, retval);

  bool activate = false;
  retval = camera.getExternalTriggering(activate);
  EXPECT_EQ(activate_expected, activate);
  EXPECT_EQ(0, retval);

  // revert back to no external triggering
  retval = camera.setExternalTriggering(false);
  EXPECT_EQ(0, retval);
}

TEST(DC1394Camera, brightness) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  double brightness_expected = 10.0;
  retval = camera.setBrightness(brightness_expected);
  EXPECT_EQ(0, retval);

  double brightness = 0.0;
  retval = camera.getBrightness(brightness);
  EXPECT_EQ(brightness_expected, brightness);
  EXPECT_EQ(0, retval);
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

TEST(DC1394Camera, exposure) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  double exposure_expected = 20.0;
  retval = camera.setExposure(exposure_expected);
  EXPECT_EQ(0, retval);

  double exposure = 0.0;
  retval = camera.getExposure(exposure);
  EXPECT_EQ(exposure_expected, exposure);
  EXPECT_EQ(0, retval);
}

TEST(DC1394Camera, shutter) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  double shutter_expected = 30.0;
  retval = camera.setShutter(shutter_expected);
  EXPECT_EQ(0, retval);

  double shutter = 0.0;
  retval = camera.getShutter(shutter);
  EXPECT_EQ(shutter_expected, shutter);
  EXPECT_EQ(0, retval);
}

TEST(DC1394Camera, gain) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  EXPECT_EQ(0, retval);

  double gain_expected = 40.0;
  retval = camera.setShutter(gain_expected);
  EXPECT_EQ(0, retval);

  double gain = 0.0;
  retval = camera.getShutter(gain);
  EXPECT_EQ(gain_expected, gain);
  EXPECT_EQ(0, retval);
}

TEST(DC1394Camera, activateSoftwareTriggeringMode) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  ASSERT_EQ(0, retval);

  retval = camera.activateSoftwareTriggeringMode();
  ASSERT_EQ(0, retval);

  // revert back to default triggering mode
  retval = camera.activateDefaultTriggeringMode();
  ASSERT_EQ(0, retval);
}

TEST(DC1394Camera, activateDefaultTriggeringMode) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  ASSERT_EQ(0, retval);

  retval = camera.activateDefaultTriggeringMode();
  ASSERT_EQ(0, retval);
}

TEST(DC1394Camera, trigger) {
  DC1394Camera camera;

  int retval = camera.configure(TEST_CONFIG_PATH);
  ASSERT_EQ(0, retval);

  retval = camera.initialize();
  ASSERT_EQ(0, retval);

  retval = camera.trigger();
  ASSERT_EQ(0, retval);
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

} // namespace atl
