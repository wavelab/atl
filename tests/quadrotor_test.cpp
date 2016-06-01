#include <iostream>

#include "awesomo/munit.h"
#include "awesomo/quadrotor.hpp"

// CONFIGS
#define POSITION_CONTROLLER_CONFIG "configs/position_controller/config.yaml"
#define CARROT_CONTROLLER_CONFIG "configs/carrot_controller/config.yaml"


// TESTS
Quadrotor *testSetup(void);
int testQuadrotor(void);
int testQuadrotorUpdatePose(void);
int testQuadrotorPositionControllerCalculate(void);
int testQuadrotorResetPositionController(void);
int testQuadrotorInitializeMission(void);
int testQuadrotorRunMission(void);


Quadrotor *testSetup(void)
{
    Quadrotor *quad;
    std::map<std::string, std::string> configs;

    // setup
	configs["position_controller"] = POSITION_CONTROLLER_CONFIG;
	configs["carrot_controller"] = CARROT_CONTROLLER_CONFIG;
	quad = new Quadrotor(configs);

	return quad;
}

int testQuadrotor(void)
{
    Quadrotor *quad;

    // setup
	quad = testSetup();

	// assert
	mu_check(quad->mission_state == IDLE_MODE);

	mu_check(quad->pose.x == 0);
	mu_check(quad->pose.y == 0);
	mu_check(quad->pose.z == 0);

	mu_check(quad->going_to.x == 0);
	mu_check(quad->going_to.y == 0);
	mu_check(quad->going_to.z == 0);

	mu_check(quad->position_controller != NULL);
	mu_check(quad->carrot_controller != NULL);

    return 0;
}

int testQuadrotorUpdatePose(void)
{
    Quadrotor *quad;
    Pose p;

    // setup
	quad = testSetup();
	p.x = 1.0;
	p.y = 2.0;
	p.z = 3.0;
	p.roll = 4.0;
	p.pitch = 5.0;
	p.yaw = 6.0;

    // test and assert
	quad->updatePose(p);
	mu_check(fltcmp(p.x, 1.0) == 0);
	mu_check(fltcmp(p.y, 2.0) == 0);
	mu_check(fltcmp(p.z, 3.0) == 0);
	mu_check(fltcmp(p.roll, 4.0) == 0);
	mu_check(fltcmp(p.pitch, 5.0) == 0);
	mu_check(fltcmp(p.yaw, 6.0) == 0);

    return 0;
}

int testQuadrotorPositionControllerCalculate(void)
{
    Quadrotor *quad;
    Pose p;
    Position setpoint;
    float dt;

    // setup
	quad = testSetup();

	p.x = 1.0;
	p.y = 1.0;
	p.z = 3.0;
	p.roll = 0.0;
	p.pitch = 0.0;
	p.yaw = 0.0;

	setpoint.x = 1.0;
	setpoint.y = 1.0;
	setpoint.z = 3.0;

	dt = 0.1;

	// test and assert
	quad->positionControllerCalculate(setpoint, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0.0f) != 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0.0f) != 0);

    return 0;
}

int testQuadrotorResetPositionController(void)
{
    Quadrotor *quad;

    // setup
	quad = testSetup();

    quad->position_controller->x.sum_error = 1.0;
    quad->position_controller->x.prev_error = 2.0;
    quad->position_controller->x.output = 3.0;

    quad->position_controller->y.sum_error = 1.0;
    quad->position_controller->y.prev_error = 2.0;
    quad->position_controller->y.output = 3.0;

    quad->position_controller->T.sum_error = 1.0;
    quad->position_controller->T.prev_error = 2.0;
    quad->position_controller->T.output = 3.0;

	// test and assert
	quad->resetPositionController();

    mu_check(fltcmp(quad->position_controller->x.sum_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.prev_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.output ,0.0) == 0);

    mu_check(fltcmp(quad->position_controller->y.sum_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.prev_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.output ,0.0) == 0);

    mu_check(fltcmp(quad->position_controller->T.sum_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.prev_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.output ,0.0) == 0);

    return 0;
}

int testQuadrotorInitializeMission(void)
{
    Quadrotor *quad;
    Pose p;

    // setup
	quad = testSetup();

	p.x = 1.0;
	p.y = 1.0;
	p.z = 3.0;
	p.roll = 0.0;
	p.pitch = 0.0;
	p.yaw = 0.0;

	quad->updatePose(p);

	// test and assert
	quad->initializeMission();

	mu_check(fltcmp(quad->carrot_controller->wp_start(0), p.x) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_start(1), p.y) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_start(2), p.z + 3) == 0);

	mu_check(fltcmp(quad->carrot_controller->wp_end(0), p.x + 5) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_end(1), p.y) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_end(2), p.z + 3) == 0);

	mu_check(quad->carrot_controller->waypoints.size() == 5);
	mu_check(quad->carrot_controller->initialized == 1);

    return 0;
}

int testQuadrotorRunMission(void)
{
    Quadrotor *quad;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    float dt;

    // setup
	quad = testSetup();

	robot_pose.x = 0.0f;
	robot_pose.y = 0.0f;
	robot_pose.z = 0.0f;
	robot_pose.roll = 0.0f;
	robot_pose.pitch = 0.0f;

    landing_zone.detected = true;
    landing_zone.x = 0.0f;
    landing_zone.y = 0.0f;
    landing_zone.z = 0.0f;

    dt = 0.1;

    // test and assert
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == TRACKING_MODE);
    mu_check(fltcmp(quad->going_to.x, robot_pose.x) == 0);
    mu_check(fltcmp(quad->going_to.y, robot_pose.y) == 0);
    mu_check(fltcmp(quad->going_to.z, 5) == 0);

	robot_pose.x = 1.0f;
	robot_pose.y = 1.0f;
	robot_pose.z = 0.0f;

    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == TRACKING_MODE);
    // mu_check(fltcmp(quad->going_to.x, robot_pose.x) == 0);
    // mu_check(fltcmp(quad->going_to.y, robot_pose.y) == 0);
    // mu_check(fltcmp(quad->going_to.z, 5) == 0);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testQuadrotor);
    mu_add_test(testQuadrotorUpdatePose);
    mu_add_test(testQuadrotorPositionControllerCalculate);
    mu_add_test(testQuadrotorInitializeMission);
    mu_add_test(testQuadrotorRunMission);
}

mu_run_tests(testSuite)
