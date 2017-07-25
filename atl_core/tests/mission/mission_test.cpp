#include "atl/atl_test.hpp"
#include "atl/mission/mission.hpp"

namespace atl {

#define TEST_CONFIG "tests/data/mission/mission.yaml"

TEST(Mission, constructor) {
  Mission mission;

  EXPECT_FALSE(mission.configured);
  EXPECT_TRUE(mission.check_waypoints);
  EXPECT_FLOAT_EQ(20, mission.waypoint_threshold);
  EXPECT_FLOAT_EQ(0.0, mission.velocity);
}

TEST(Mission, configure) {
  Mission mission;

  int retval = mission.configure(TEST_CONFIG);

  EXPECT_EQ(0.0, retval);

  EXPECT_TRUE(mission.check_waypoints);
  EXPECT_FLOAT_EQ(20, mission.waypoint_threshold);

  EXPECT_FLOAT_EQ(0.5, mission.velocity);
  EXPECT_EQ(4, (size_t) mission.waypoints.size());

  EXPECT_FLOAT_EQ(43.474024, mission.waypoints[0].latitude);
  EXPECT_FLOAT_EQ(-80.540287, mission.waypoints[0].longitude);
  EXPECT_FLOAT_EQ(340.0, mission.waypoints[0].altitude);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[0].staytime);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[0].heading);

  EXPECT_FLOAT_EQ(43.474078, mission.waypoints[1].latitude);
  EXPECT_FLOAT_EQ(-80.540161, mission.waypoints[1].longitude);
  EXPECT_FLOAT_EQ(340.0, mission.waypoints[1].altitude);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[1].staytime);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[1].heading);

  EXPECT_FLOAT_EQ(43.473957, mission.waypoints[2].latitude);
  EXPECT_FLOAT_EQ(-80.540073, mission.waypoints[2].longitude);
  EXPECT_FLOAT_EQ(340.0, mission.waypoints[2].altitude);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[2].staytime);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[2].heading);

  EXPECT_FLOAT_EQ(43.473929, mission.waypoints[3].latitude);
  EXPECT_FLOAT_EQ(-80.540173, mission.waypoints[3].longitude);
  EXPECT_FLOAT_EQ(340.0, mission.waypoints[3].altitude);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[3].staytime);
  EXPECT_FLOAT_EQ(0.0, mission.waypoints[3].heading);
}

}  // namespace atl
