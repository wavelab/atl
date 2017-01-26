#include "awesomo_core/awesomo_test.hpp"
#include "awesomo_core/planning/utils.hpp"

namespace awesomo {

TEST(BezierCubicCurve, test) {
  Vec2 p0;
  Vec2 p1;
  Vec2 p2;
  Vec2 p3;
  std::vector<Vec2> bezier_points;

  // setup
  p0 << 0.0, 40.0;
  p1 << 5.0, 30.0;
  p2 << 5.0, 10.0;
  p3 << 3.0, 0.0;

  // create bezier curve
  bezier_points = bezier_cubic_curve(p0, p1, p2, p3);
  for (int i = 1; i < bezier_points.size(); i++) {
    ASSERT_TRUE(bezier_points[i](1) < bezier_points[i - 1](1));
  }
}

}  // end of awesomo namespace
