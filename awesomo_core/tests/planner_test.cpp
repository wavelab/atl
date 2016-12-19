#include "awesomo_core/munit.h"
#include "awesomo_core/planner.hpp"

namespace awesomo {

// TESTS
int testBezierCubicCurve(void);


int testBezierCubicCurve(void) {
  Eigen::Vector2d p0;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
  Eigen::Vector2d p3;
  std::vector<Eigen::Vector2d> bezier_points;

  // setup
  p0 << 0.0, 40.0;
  p1 << 5.0, 30.0;
  p2 << 5.0, 10.0;
  p3 << 3.0, 0.0;

  // create bezier curve
  bezier_points = bezier_cubic_curve(p0, p1, p2, p3);
  for (int i = 1; i < bezier_points.size(); i++) {
    mu_check(bezier_points[i](1) < bezier_points[i - 1](1));
  }

  return 0;
}

}  // end of awesomo namepsace

void testSuite(void) {
  mu_add_test(awesomo::testBezierCubicCurve);
}

mu_run_tests(testSuite)
