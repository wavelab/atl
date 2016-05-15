#include "munit.h"
#include "controller.hpp"


#define CARROT_CONTROLLER_CONFIG "configs/carrot_controller/config.yaml"
#define POSITION_CONTROLLER_CONFIG "configs/position_controller/config.yaml"


// TESTS
int testCarrotController(void);
int testCarrotControllerClosestPoint(void);
int testCarrotControllerCalculateCarrotPoint(void);
int testCarrotControllerWaypointReached(void);
int testCarrotControllerUpdate(void);
int testPositionControllerLoadConfig(void);
int testAttitudeController(void);


int testCarrotController(void)
{
    CarrotController *controller;

    controller = new CarrotController(CARROT_CONTROLLER_CONFIG);
    mu_check(controller->initialized != 0);
    mu_check(controller->look_ahead_dist != 0);
    mu_check(controller->wp_threshold != 0);
    mu_check(controller->waypoints.size() != 0);

    return 0;
}

int testCarrotControllerClosestPoint(void)
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

int testCarrotControllerCalculateCarrotPoint(void)
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

int testCarrotControllerWaypointReached(void)
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

int testCarrotControllerUpdate(void)
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
    // wp << 15, 10, 0;
    // controller.waypoints.push_back(wp);
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

    return 0;
}

int testPositionControllerLoadConfig(void)
{
    PositionController *controller;
    controller = new PositionController(POSITION_CONTROLLER_CONFIG);

    mu_check(controller->roll == 0);
    mu_check(controller->pitch == 0);
    mu_check(controller->throttle == 0);

    mu_print("x.setpoint: %f\n", controller->x.setpoint);
    mu_print("x.min: %f\n", controller->x.min);
    mu_print("x.max: %f\n", controller->x.max);
    mu_print("x.k_p: %f\n", controller->x.k_p);
    mu_print("x.k_i: %f\n", controller->x.k_i);
    mu_print("x.k_d: %f\n\n", controller->x.k_d);

    mu_print("y.setpoint: %f\n", controller->y.setpoint);
    mu_print("y.min: %f\n", controller->y.min);
    mu_print("y.max: %f\n", controller->y.max);
    mu_print("y.k_p: %f\n", controller->y.k_p);
    mu_print("y.k_i: %f\n", controller->y.k_i);
    mu_print("y.k_d: %f\n\n", controller->y.k_d);

    mu_print("T.setpoint: %f\n", controller->T.setpoint);
    mu_print("T.min: %f\n", controller->T.min);
    mu_print("T.max: %f\n", controller->T.max);
    mu_print("T.k_p: %f\n", controller->T.k_p);
    mu_print("T.k_i: %f\n", controller->T.k_i);
    mu_print("T.k_d: %f\n\n", controller->T.k_d);

    return 0;
}

int testAttitudeController(void)
{
    AttitudeController *controller;
    controller = new AttitudeController();

    Motors *motors;
    RCControl *rc_control;
    float roll;
    float pitch;
    float throttle;

    motors = new Motors();
    rc_control = new RCControl();

    while (1) {
        rc_control->update();

        roll = (rc_control->ch1 - 1500.0) / (2000.0 - 1500.0);
        pitch = (rc_control->ch2 - 1500.0) / (2000.0 - 1500.0);
        throttle = (rc_control->ch3 - 1092) / (1900.0 - 1092.0);

        motors->set_throttle(0, throttle - roll + pitch);
        motors->set_throttle(1, throttle + roll - pitch);
        motors->set_throttle(2, throttle + roll + pitch);
        motors->set_throttle(3, throttle - roll - pitch);
    }

    return 0;
}

void testSuite(void)
{
    mu_add_test(testCarrotController);
    mu_add_test(testCarrotControllerClosestPoint);
    mu_add_test(testCarrotControllerCalculateCarrotPoint);
    mu_add_test(testCarrotControllerWaypointReached);
    // mu_add_test(testCarrotControllerUpdate);
    mu_add_test(testPositionControllerLoadConfig);
    mu_add_test(testAttitudeController);
}

mu_run_tests(testSuite)
