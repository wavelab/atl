#include <gtest/gtest.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/planning/planning.hpp"


TEST(BezierCubicCurve, test) {
  awesomo::Vec2 p0;
  awesomo::Vec2 p1;
  awesomo::Vec2 p2;
  awesomo::Vec2 p3;
  std::vector<awesomo::Vec2> bezier_points;

  // setup
  p0 << 0.0, 40.0;
  p1 << 5.0, 30.0;
  p2 << 5.0, 10.0;
  p3 << 3.0, 0.0;

  // create bezier curve
  bezier_points = awesomo::bezier_cubic_curve(p0, p1, p2, p3);
  for (int i = 1; i < bezier_points.size(); i++) {
    ASSERT_TRUE(bezier_points[i](1) < bezier_points[i - 1](1));
  }
}
