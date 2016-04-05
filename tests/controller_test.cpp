#include "awesomo/munit.h"
#include "awesomo/controller.hpp"


// TESTS
int testClosestPoint(void);
int testCalculateCarrotPoint(void);
int testWaypointReached(void);
int testUpdate(void);


int testClosestPoint(void)
{
    CarrotController controller;
    Eigen::Vector3d wp_start;
    Eigen::Vector3d wp_end;
    Eigen::Vector3d position;
    Eigen::Vector3d point;
    Eigen::Vector3d expected;

    // setup
    wp_start << 1, 1, 0;
    wp_end << 10, 10, 0;
    position << 5, 8, 0;
    expected << 6.5, 6.5, 0.0;

    // test and assert
    point = controller.closestPoint(position, wp_start, wp_end);
    mu_check(point == expected);

    // std::cout << point << std::endl;
    // std::cout << expected << std::endl;
    return 0;
}

int testCalculateCarrotPoint(void)
{
    CarrotController controller;

    double r;
    Eigen::Vector3d wp_start;
    Eigen::Vector3d wp_end;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot_point;
    Eigen::Vector3d expected;

    // setup
    r = 2.0;
    wp_start << 1, 1, 0;
    wp_end << 10, 10, 0;
    position << 5, 8, 0;
    expected << 6.5, 6.5, 0.0;

    // test and assert
    carrot_point = controller.calculateCarrotPoint(
        position,
        r,
        wp_start,
        wp_end
    );
    // mu_check(point == expected);

    std::cout << carrot_point << std::endl;
    // std::cout << expected << std::endl;


    return 0;
}

int testWaypointReached(void)
{
    CarrotController controller;
    int retval;
    double threshold;
    Eigen::Vector3d waypoint;
    Eigen::Vector3d position;

    // setup
    threshold = 1.0;
    waypoint << 5, 8, 0;
    position << 5, 8, 0;

    // test and assert
    retval = controller.waypointReached(position, waypoint, threshold);
    mu_check(retval == 1);

    position << 10, 10, 0;
    retval = controller.waypointReached(position, waypoint, threshold);
    mu_check(retval == 0);

    return 0;
}

int testUpdate(void)
{
    CarrotController controller;
    int retval;
    Eigen::Vector3d wp;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    // setup
    position << 2, 2, 0;
    wp << 0, 0, 0;
    controller.wp_start = wp;
    controller.waypoints.push_back(wp);
    wp << 10, 10, 0;
    controller.wp_end = wp;
    controller.waypoints.push_back(wp);
    wp << 15, 10, 0;
    controller.waypoints.push_back(wp);
    controller.look_ahead_dist = 1;
    controller.wp_threshold = 0.1;
    controller.initialized = 1;

    // test and assert
    std::ofstream outfile;
    outfile.open("update.dat");

    for (int i = 0; i < 40; i++) {
        // carrot update
        retval = controller.update(position, carrot);
        mu_check(retval == 1);

        // record
        outfile << controller.wp_start(0) << ", ";
        outfile << controller.wp_start(1) << ", ";
        outfile << controller.wp_start(2) << std::endl;

        outfile << controller.wp_end(0) << ", ";
        outfile << controller.wp_end(1) << ", ";
        outfile << controller.wp_end(2) << std::endl;

        outfile << position(0) << ", ";
        outfile << position(1) << ", ";
        outfile << position(2) << std::endl;

        outfile << carrot(0) << ", ";
        outfile << carrot(1) << ", ";
        outfile << carrot(2) << std::endl;
        outfile << std::endl;

        // update position
        position(0) = position(0) + 0.5;
        position(1) = position(1) + 0.5;
    }

    outfile.close();
}

void testSuite(void)
{
    mu_add_test(testClosestPoint);
    mu_add_test(testCalculateCarrotPoint);
    mu_add_test(testWaypointReached);
    mu_add_test(testUpdate);
}

mu_run_tests(testSuite)
