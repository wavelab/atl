#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/quadrotor/modes/tracking_mode.hpp"

#define TEST_CONFIG "tests/configs/quadrotor/modes/tracking_mode.yaml"


namespace awesomo {

TEST(TrackingMode, constructor) {
  TrackingMode mode;

  ASSERT_FALSE(mode.configured);

  ASSERT_FLOAT_EQ(0.0, mode.min_track_time);
  ASSERT_FLOAT_EQ(0.0, mode.target_lost_threshold);

  ASSERT_TRUE(mode.target_losted);
  ASSERT_FALSE(mode.target_detected);
  ASSERT_FLOAT_EQ(0.0, mode.target_bpf(0));
  ASSERT_FLOAT_EQ(0.0, mode.target_bpf(1));
  ASSERT_FLOAT_EQ(0.0, mode.target_bpf(2));
}

TEST(TrackingMode, configure) {
  TrackingMode mode;

  mode.configure(TEST_CONFIG);

  ASSERT_TRUE(mode.configured);

  ASSERT_FLOAT_EQ(1.0, mode.min_track_time);
  ASSERT_FLOAT_EQ(2.0, mode.target_lost_threshold);

  ASSERT_TRUE(mode.target_losted);
  ASSERT_FALSE(mode.target_detected);
  ASSERT_FLOAT_EQ(0.0, mode.target_bpf(0));
  ASSERT_FLOAT_EQ(0.0, mode.target_bpf(1));
  ASSERT_FLOAT_EQ(0.0, mode.target_bpf(2));
}

TEST(TrackingMode, updateTargetPosition) {
  TrackingMode mode;
  Vec3 position;

  mode.configure(TEST_CONFIG);
  position << 1.0, 2.0, 3.0;
  mode.updateTargetPosition(position, true);

  ASSERT_FALSE(mode.target_losted);
  ASSERT_TRUE(mode.target_detected);
  ASSERT_FLOAT_EQ(1.0, mode.target_bpf(0));
  ASSERT_FLOAT_EQ(2.0, mode.target_bpf(1));
  ASSERT_FLOAT_EQ(3.0, mode.target_bpf(2));
  ASSERT_TRUE(mtoc(&mode.target_last_seen) < 1);
}

TEST(TrackingMode, update) {
  TrackingMode mode;
  Vec3 position;

  mode.configure(TEST_CONFIG);
  position << 1.0, 2.0, 3.0;
  mode.updateTargetPosition(position, true);
  ASSERT_TRUE(mode.target_detected);
  ASSERT_FALSE(mode.target_losted);

  sleep(3);
  mode.update();
  ASSERT_TRUE(mode.target_losted);
}

}  // end of awesomo namespace
