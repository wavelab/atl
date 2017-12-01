#include "atl/planning/utils.hpp"

namespace atl {

std::vector<Eigen::Vector2d> bezier_cubic_curve(Eigen::Vector2d p0,
                                                Eigen::Vector2d p1,
                                                Eigen::Vector2d p2,
                                                Eigen::Vector2d p3) {
  Eigen::Vector2d p;
  std::vector<Eigen::Vector2d> bezier_points;
  int segment_count;
  double t;

  // setup
  segment_count = 20;

  // calculate bezier points
  for (int i = 1; i < segment_count + 1; i++) {
    double u = 1 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;

    t = i / (double) segment_count;
    p << 0.0, 0.0;

    // first term
    p(0) = uuu * p0(0);
    p(1) = uuu * p0(1);

    // second term
    p(0) += 3 * uu * t * p1(0);
    p(1) += 3 * uu * t * p1(1);

    // third term
    p(0) += 3 * u * tt * p2(0);
    p(1) += 3 * u * tt * p2(1);

    // fourth term
    p(0) += ttt * p3(0);
    p(1) += ttt * p3(1);

    // append to vector of other bezier points
    bezier_points.push_back(p);
  }

  return bezier_points;
}

} // namespace atl
