#include "awesomo_core/munit.h"
#include "awesomo_core/controller.hpp"


#define POSITION_CONTROLLER_CONFIG "configs/position_controller/config.yaml"


// TESTS
int testPositionControllerLoadConfig(void);
int testPositionControllerPidCalculate(void);


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

int testPositionControllerPidCalculate(void)
{
    PositionController *controller;
    controller = new PositionController(POSITION_CONTROLLER_CONFIG);

    Eigen::Vector3d setpoint;
    Pose robot_pose;
    float yaw_setpoint;
    float dt;


    // check hovering PID output
    setpoint << 0, 0, 0;
    robot_pose = Pose(0, 0, 0, 0, 0, 0);
    yaw_setpoint = 0;
    dt = 0.1;

    controller->calculate(setpoint, robot_pose, yaw_setpoint, dt);

    std::cout << controller->roll << "\t"
              << controller->pitch << "\t"
              << controller->throttle << std::endl;

    mu_check(controller->roll == 0);
    mu_check(controller->pitch == 0.);
    mu_check(fltcmp(controller->throttle, controller->hover_throttle) == 0);

    // check moving towards and x location
    setpoint << 1, 0, 0;
    robot_pose = Pose(0, 0, 0, 0, 0, 0);
    yaw_setpoint = 0;
    dt = 0.1;

    controller->reset();
    controller->calculate(setpoint, robot_pose, yaw_setpoint, dt);
    mu_check(controller->roll == 0.0);
    mu_check(controller->pitch < 0.0);
    mu_check(controller->throttle > controller->hover_throttle);

    // check moving towards and x and y location
    setpoint << 1, -1, 0;
    robot_pose = Pose(0, 0, 0, 0, 0, 0);
    yaw_setpoint = 0;
    dt = 0.1;

    controller->reset();
    controller->calculate(setpoint, robot_pose, yaw_setpoint, dt);
    mu_check(controller->roll < 0.0);
    mu_check(controller->pitch < 0.0);
    mu_check(controller->throttle > controller->hover_throttle);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testPositionControllerLoadConfig);
    mu_add_test(testPositionControllerPidCalculate);
}

mu_run_tests(testSuite)
