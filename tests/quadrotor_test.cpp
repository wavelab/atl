#include <iostream>

#include "awesomo/munit.h"
#include "awesomo/quadrotor.hpp"

// CONFIGS
#define QUADROTOR_CONFIG "configs/quadrotor/config.yaml"
#define POSITION_CONTROLLER_CONFIG "configs/position_controller/config.yaml"
#define CARROT_CONTROLLER_CONFIG "configs/carrot_controller/config.yaml"


// TESTS
Quadrotor *testSetup(void);
int testQuadrotor(void);
int testQuadrotorUpdatePose(void);
int testQuadrotorPositionControllerCalculate(void);
int testQuadrotorResetPositionController(void);
int testQuadrotorInitializeCarrotController(void);
int testQuadrotorRunCarrotMode(void);
int testQuadrotorRunDiscoveryMode(void);
int testQuadrotorRunTrackingModeBPF(void);
int testQuadrotorRunLandingMode(void);
int testQuadrotorRunMission(void);


Quadrotor *testSetup(void)
{
    Quadrotor *quad;
    std::map<std::string, std::string> configs;

    // setup
	configs["quadrotor"] = QUADROTOR_CONFIG;
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

	mu_check(quad->world_pose.position(0) == 0);
	mu_check(quad->world_pose.position(1) == 0);
	mu_check(quad->world_pose.position(2) == 0);

	mu_check(fltcmp(quad->hover_height, 0.0) == 0);

	mu_check(quad->landing_zone_belief == 0);

	mu_check(quad->estimator_initialized == false);

	mu_check(quad->position_controller != NULL);
	mu_check(quad->carrot_controller != NULL);

    return 0;
}

// int testQuadrotorUpdatePose(void)
// {
//     Quadrotor *quad;
//     Pose p;
//     Eigen::Quaterniond test_quat;
//
//     // setup
// 	quad = testSetup();
// 	x = 1.0;
// 	y = 2.0;
// 	z = 3.0;
// 	roll = 4.0;
// 	pitch = 5.0;
// 	yaw = 6.0;
//
// 	p = Pose(roll, pitch, yaw, x, y, z);
// 	kkk
//     // test and assert
// 	quad->updatePose(p);
// 	mu_check(fltcmp(p.position(0), 1.0) == 0);
// 	mu_check(fltcmp(p.position(1), 2.0) == 0);
// 	mu_check(fltcmp(p.position(2), 3.0) == 0);
// 	mu_check(fltcmp(p.q.x(), 4.0) == 0);
// 	mu_check(fltcmp(p.position(pitch, 5.0) == 0);
// 	mu_check(fltcmp(p.position(yaw, 6.0) == 0);
//
//     return 0;
// }

int testQuadrotorPositionControllerCalculate(void)
{
    Quadrotor *quad;
    Pose p;
    Eigen::Vector3d setpoint;
    float dt;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    // setup
	quad = testSetup();
    x = 1.0;
    y = 1.0;
    z = 3.0;
	roll = 0;
	pitch = 0;
	yaw = 0;
    p = Pose(roll, pitch, yaw,  1, 1, 3);

    setpoint = Eigen::Vector3d(x, y, z);

	dt = 0.1;

	// test no correction
	quad->positionControllerCalculate(setpoint, p, 0, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);

	// test correction point to left
	x = 1.0;
	y = 2.0;
	z = 3.0;
	p.position << x, y, z;

    x = 1.0;
    y = 1.0;
    z = 3.0;
    setpoint = Eigen::Vector3d(x, y, z);

	quad->positionControllerCalculate(setpoint, p, 0, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0.0f) < 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);

    quad->position_controller->reset();
	// test correction point to right
	x = 1.0;
	y = 0.0;
	z = 3.0;
	p.position << x, y, z;

    x = 1.0;
    y = 1.0;
    z = 3.0;
    setpoint = Eigen::Vector3d(x, y, z);

	quad->positionControllerCalculate(setpoint, p, 0, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0.0f) > 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);

    quad->position_controller->reset();
	// test correction point in front
	x = 0.0;
	y = 1.0;
	z = 3.0;
	p.position << x, y, z;

    x = 1.0;
    y = 1.0;
    z = 3.0;
    setpoint = Eigen::Vector3d(x, y, z);

	quad->positionControllerCalculate(setpoint, p, 0, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0.0f) > 0);


    quad->position_controller->reset();
	// test correction point behind
	x = 0.0;
	y = 1.0;
	z = 3.0;
	p.position << x, y, z;

    x = 1.0;
    y = 1.0;
    z = 3.0;
    setpoint = Eigen::Vector3d(x, y, z);

	quad->positionControllerCalculate(setpoint, p, 0, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0.0f) > 0);
    return 0;
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

int testQuadrotorInitializeCarrotController(void)
{
    Quadrotor *quad;
    Pose p;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;

    // setup
	quad = testSetup();

	x = 1.0;
	y = 1.0;
	z = 3.0;
	roll = 0.0;
	pitch = 0.0;
	yaw = 0.0;

	p = Pose(roll, pitch, yaw, x, y, z);

	quad->world_pose = p;

	// test and assert
	quad->initializeCarrotController();
    // std::cout << "carrot wp start \t " << quad->carrot_controller->wp_start(2) << std::endl;
	mu_check(fltcmp(quad->carrot_controller->wp_start(0), p.position(0) == 0));
	mu_check(fltcmp(quad->carrot_controller->wp_start(1), p.position(1) == 0));
	mu_check(fltcmp(quad->carrot_controller->wp_start(2), p.position(2) + 3) == 0);

	mu_check(fltcmp(quad->carrot_controller->wp_end(0), p.position(0) + 5) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_end(1), p.position(1)) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_end(2), p.position(2) + 3) == 0);

	mu_check(quad->carrot_controller->waypoints.size() == 5);
	mu_check(quad->carrot_controller->initialized == 1);

    return 0;
}

// int testQuadrotorRunCarrotMode(void)
// {
//     Quadrotor *quad;
//     Pose p;
//     Pose robot_pose;
//     Eigen::Vector3d cmd_position;
//     float x_waypoints[5] = {0.0, 5.0, 5.0, 0.0, 0.0};
//     float y_waypoints[5] = {0.0, 0.0, 5.0, 5.0, 0.0};
//     float x;
//     float y;
//     float z;
//     float roll;
//     float pitch;
//     float yaw;
//
//     // setup
// 	quad = testSetup();
//
// 	x = 0.0;
// 	y = 0.0;
// 	z = 3.0;
// 	roll = 0.0;
// 	pitch = 0.0;
// 	yaw = 0.0;
//
// 	p = Pose(roll, pitch, yaw, x, y, z);
//
// 	quad->world_pose = p;
// 	quad->initializeCarrotController();
//
// 	// test and assert
// 	for (int i = 0; i < 5; i++) {
//         // (i + 1)-th waypoint
//         robot_pose.position(0) = x_waypoints[i];
//         robot_pose.position(1) = y_waypoints[i];
//         robot_pose.position(2) = 6.0;
//         cmd_position = quad->runCarrotMode(robot_pose, 0.1);
//
//         if (i < 3) {
//             // check waypoint 2 to 4
//             mu_check(quad->carrot_controller->waypoints.size() == (5 - i));
//         } else {
//             // check waypoint 5
//             mu_check(quad->carrot_controller->waypoints.size() == 2);
//         }
//
//         mu_print("pos: %f %f %f\n", robot_pose.position(0),
//             robot_pose.position(1), robot_pose.position(2));
//         mu_print("cmd_position: %f %f %f\n", cmd_position(0),
//             cmd_position(1), cmd_position(1));
//         mu_print("waypoints: %d\n\n", (int) quad->carrot_controller->waypoints.size());
// 	}
//
//     // check if hover mode activated
// 	mu_check(quad->mission_state == HOVER_MODE);
// 	mu_check(fltcmp(quad->hover_point->position(0),
// 	    quad->carrot_controller->wp_end(0)) == 0);
// 	mu_check(fltcmp(quad->hover_point->position(1),
// 	    quad->carrot_controller->wp_end(1)) == 0);
// 	mu_check(fltcmp(quad->hover_height,
// 	    quad->carrot_controller->wp_end(2)) == 0);
//
//     return 0;
// }

int testQuadrotorRunDiscoveryMode(void)
{
    Quadrotor *quad;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    Eigen::Vector3d cmd_position;


    // setup
	quad = testSetup();

	// test transition to tracking mode
	robot_pose.position(0) = 1.0;
	robot_pose.position(1) = 2.0;
	robot_pose.position(2) = 3.0;

	landing_zone.detected = true;
	landing_zone.position(0) = 0.0;
	landing_zone.position(1) = 0.0;
	landing_zone.position(2) = 0.0;

    cmd_position = quad->runDiscoverMode(robot_pose, landing_zone);
    mu_check(quad->mission_state == TRACKING_MODE);

    // test hover during discover mode
	landing_zone.detected = false;
    cmd_position = quad->runDiscoverMode(robot_pose, landing_zone);

    // check quad->tracking_start was instanciated within 2 seconds ago
    mu_check(quad->tracking_start <= time(NULL));
    mu_check(quad->tracking_start >= time(NULL) - 2);

    return 0;
}

int testQuadrotorRunTrackingModeBPF(void)
{
    Quadrotor *quad;
    float dt;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    Eigen::Vector3d cmd_position;

    // setup
	quad = testSetup();
	dt = 0.1;

	// test tracking mode - time step 1
	landing_zone.detected = true;
    robot_pose.position(0) = 0.0;
    robot_pose.position(1) = 0.0;
    robot_pose.position(2) = 0.0;

    landing_zone.position(0) = 1.0;
    landing_zone.position(1) = 2.0;
    landing_zone.position(2) = 3.0;

	quad->runDiscoverMode(robot_pose, landing_zone);
	// cmd_position = quad->runTrackingModeBPF(landing_zone, dt);
    // mu_print("cmd_position: %f %f %f\n",
    //     cmd_position(0),
    //     cmd_position(1),
    //     cmd_position(2));
    //
	// // test tracking mode - time step 2
    // landing_zone.position(0) = 1.1;
    // landing_zone.position(1) = 2.0;
    // landing_zone.position(2) = 3.0;
    //
	// cmd_position = quad->runTrackingModeBPF(landing_zone, dt);
    // mu_print("cmd_position: %f %f %f\n",
    //     cmd_position(0),
    //     cmd_position(1),
    //     cmd_position(2));
    //
	// // test tracking mode - time step 3
    // landing_zone.position(0) = 1.3;
    // landing_zone.position(1) = 2.0;
    // landing_zone.position(2) = 3.0;
    //
	// cmd_position = quad->runTrackingModeBPF(landing_zone, dt);
    // mu_print("cmd_position: %f %f %f\n",
    //     cmd_position(0),
    //     cmd_position(1),
    //     cmd_position(2));

    return 0;
}

int testQuadrotorRunLandingMode(void)
{
    Quadrotor *quad;
    float dt;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    Eigen::Vector3d cmd_position;

    // setup
	quad = testSetup();
	dt = 0.1;

	robot_pose.position(0) = 1.0;
	robot_pose.position(1) = 2.0;
	robot_pose.position(2) = 3.0;

	landing_zone.detected = true;
	landing_zone.position(0) = 0.0;
	landing_zone.position(1) = 0.0;
	landing_zone.position(2) = 4.0;

	quad->runDiscoverMode(robot_pose, landing_zone);
	quad->mission_state = LANDING_MODE;
    quad->hover_height= 6.0;

	// test landing mode - lower height
    quad->height_last_updated = time(NULL) - 2;
	cmd_position = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.2) == 0);

    // test landing mode - do not lower height
    quad->height_last_updated = time(NULL);
    landing_zone.position(0) = 0.3;
    landing_zone.position(1) = 0.3;
	cmd_position = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.2) == 0);

	// test landing mode - increase height
    quad->height_last_updated = time(NULL) - 2;
    landing_zone.position(0) = 0.51;
    landing_zone.position(1) = 0.51;
	cmd_position = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.2 * 1.2) == 0);

    // test landing mode - do not increase height
    quad->height_last_updated = time(NULL);
    landing_zone.position(0) = 0.5;
    landing_zone.position(1) = 0.5;
	cmd_position = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.2 * 1.2) == 0);

    // test landing mode - kill engines
    landing_zone.position << 0.19, 0.19, 0.19;
	cmd_position = quad->runLandingMode(robot_pose, landing_zone, dt);
	mu_check(quad->mission_state == MISSION_ACCOMPLISHED);

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

    landing_zone.detected = true;
    landing_zone.position(0) = 0.0f;
    landing_zone.position(1) = 0.0f;
    landing_zone.position(2) = 0.0f;

    dt = 0.1;

    // test IDLE_MODE
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == DISCOVER_MODE);
    mu_check(quad->hover_height > 0);

    // test TRACKER_MODE
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == TRACKING_MODE);
    // mu_check(fltcmp(quad->hover_point->position(0), robot_pose.position(0)) == 0);
    // mu_check(fltcmp(quad->hover_point->position(1), robot_pose.position(1)) == 0);
    mu_check(quad->hover_height > 0);

    // Does not seem to be enabled at the moment
    // quad->tracking_start = quad->tracking_start - 100;  // emulate tracking for 100 seconds
    // quad->runMission(robot_pose, landing_zone, dt);
    // mu_check(quad->mission_state == LANDING_MODE);

    // test LANDING_MODE
    landing_zone.position << 0.19, 0.19, 0.19;
    // set detected landing zone within disarm range
	quad->runLandingMode(robot_pose, landing_zone, dt);
	mu_check(quad->mission_state == MISSION_ACCOMPLISHED);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testQuadrotor);
    mu_add_test(testQuadrotorPositionControllerCalculate);
    mu_add_test(testQuadrotorInitializeCarrotController);
    // mu_add_test(testQuadrotorRunCarrotMode);
    mu_add_test(testQuadrotorRunDiscoveryMode);
    mu_add_test(testQuadrotorRunTrackingModeBPF);
    mu_add_test(testQuadrotorRunLandingMode);
    mu_add_test(testQuadrotorRunMission);
    // mu_add_test(testQuadrotorUpdatePose);
}

mu_run_tests(testSuite)
