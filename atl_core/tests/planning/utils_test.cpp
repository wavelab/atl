#include "atl/planning/utils.hpp"
#include "atl/atl_test.hpp"

namespace atl {

TEST(BezierCubicCurve, test) {
  // setup
  Vec2 p0{0.0, 40.0};
  Vec2 p1{5.0, 30.0};
  Vec2 p2{5.0, 10.0};
  Vec2 p3{3.0, 0.0};

  // create bezier curve
  std::vector<Vec2> bezier_points = bezier_cubic_curve(p0, p1, p2, p3);
  for (size_t i = 1; i < bezier_points.size(); i++) {
    EXPECT_TRUE(bezier_points[i](1) < bezier_points[i - 1](1));
  }
}

}  // namespace atl
