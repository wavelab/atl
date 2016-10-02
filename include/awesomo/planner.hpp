#ifndef __PLANNER_HPP__
#define __PLANNER_HPP__

#include <vector>

#include <Eigen/Dense>


// FUNCTIONS
std::vector<Eigen::Vector2d> bezier_cubic_curve(
    Eigen::Vector2d pt0,
    Eigen::Vector2d pt1,
    Eigen::Vector2d pt2,
    Eigen::Vector2d pt3
);


#endif
