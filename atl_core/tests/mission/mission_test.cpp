#include "atl/atl_test.hpp"
#include "atl/mission/mission.hpp"

namespace atl {

#define TEST_CONFIG "tests/data/mission/mission.yaml"

TEST(Mission, constructor) {
  Mission mission;

  EXPECT_FALSE(mission.configured);
  EXPECT_FLOAT_EQ(0.5, mission.max_velocity);
  EXPECT_FLOAT_EQ(5.0, mission.altitude);
}

TEST(Mission, configure) {
  Mission mission;

  int retval = mission.configure(TEST_CONFIG);

  EXPECT_EQ(0.0, retval);
  EXPECT_FLOAT_EQ(10, mission.altitude);

  EXPECT_FLOAT_EQ(43.474024, mission.waypoints[0].latitude);
  EXPECT_FLOAT_EQ(-80.540287, mission.waypoints[0].longitude);
  EXPECT_FLOAT_EQ(43.474078, mission.waypoints[1].latitude);
  EXPECT_FLOAT_EQ(-80.540161, mission.waypoints[1].longitude);
  EXPECT_FLOAT_EQ(43.473957, mission.waypoints[2].latitude);
  EXPECT_FLOAT_EQ(-80.540073, mission.waypoints[2].longitude);
  EXPECT_FLOAT_EQ(43.473929, mission.waypoints[3].latitude);
  EXPECT_FLOAT_EQ(-80.540173, mission.waypoints[3].longitude);
}

}  // namespace atl
