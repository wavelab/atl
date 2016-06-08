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
int testQuadrotorRunIdleMode(void);
int testQuadrotorRunHoverMode(void);
int testQuadrotorInitializeCarrotController(void);
int testQuadrotorRunCarrotMode(void);
int testQuadrotorRunDiscoveryMode(void);
int testQuadrotorRunTrackingMode(void);
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
	// // tracking
	mu_check(quad->tracking_config->offset_x >= 0);
	mu_check(quad->tracking_config->offset_y >= 0);
	mu_check(quad->tracking_config->offset_z >= 0);

	// // landing
	mu_check(quad->landing_config->period >= 1);
	mu_check(quad->landing_config->descend_multiplier >= 0);
	mu_check(quad->landing_config->descend_multiplier < 1);
	mu_check(quad->landing_config->recover_multiplier > 1);
	mu_check(quad->landing_config->x_cutoff > 0);
	mu_check(quad->landing_config->y_cutoff > 0);
	mu_check(quad->landing_config->z_cutoff > 0);
	mu_check(quad->landing_config->x_cutoff < 1);
	mu_check(quad->landing_config->y_cutoff < 1);
	mu_check(quad->landing_config->z_cutoff < 1);
	mu_check(quad->landing_config->belief_threshold > 1);

	// // states
	mu_check(quad->mission_state == IDLE_MODE);

	mu_check(quad->pose.x == 0);
	mu_check(quad->pose.y == 0);
	mu_check(quad->pose.z == 0);

	mu_check(quad->hover_point->initialized == true);
	mu_check(fltcmp(quad->hover_point->x, 0.0) == 0);
	mu_check(fltcmp(quad->hover_point->y, 0.0) == 0);
	mu_check(quad->hover_point->z > 0.0);

    // controllers
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
	quad->positionControllerCalculate(setpoint, p, dt);
	mu_check(fltcmp(quad->position_controller->roll, 0) == 0);
	mu_check(fltcmp(quad->position_controller->pitch, 0) == 0);

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

int testQuadrotorRunIdleMode(void)
{
    Quadrotor *quad;
    Pose robot_pose;

    // setup
	quad = testSetup();
	robot_pose.x = 1.0;
	robot_pose.y = 2.0;
	robot_pose.z = 3.0;

	// test and assert
	quad->runIdleMode(robot_pose);
	mu_check(quad->mission_state == DISCOVER_MODE);
	mu_check(quad->hover_point->initialized == true);
	mu_check(fltcmp(quad->hover_point->x, robot_pose.x) == 0);
	mu_check(fltcmp(quad->hover_point->y, robot_pose.y) == 0);
	mu_check(quad->hover_point->z > 0);

    return 0;
}

int testQuadrotorRunHoverMode(void)
{
    Quadrotor *quad;
    Pose robot_pose;
    Position cmd;
    float dt;

    // setup
	quad = testSetup();
	robot_pose.x = 1.0;
	robot_pose.y = 2.0;
	robot_pose.z = 3.0;
	dt = 0.1;

	// test and assert
	quad->hover_point->initialized = false;  // uninitalize it so it uses robot_pose
	cmd = quad->runHoverMode(robot_pose, dt);

    mu_check(quad->hover_point->initialized == true);
    mu_check(fltcmp(quad->hover_point->x, robot_pose.x) == 0);
    mu_check(fltcmp(quad->hover_point->y, robot_pose.y) == 0);
    mu_check(fltcmp(quad->hover_point->z, robot_pose.z) == 0);

    mu_check(fltcmp(cmd.x, robot_pose.x) == 0);
    mu_check(fltcmp(cmd.y, robot_pose.y) == 0);
    mu_check(fltcmp(cmd.z, robot_pose.z) == 0);

    return 0;
}

int testQuadrotorInitializeCarrotController(void)
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
	quad->initializeCarrotController();

	mu_check(fltcmp(quad->carrot_controller->wp_start(0), p.x) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_start(1), p.y) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_start(2), quad->hover_point->z) == 0);

	mu_check(fltcmp(quad->carrot_controller->wp_end(0), p.x + 5) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_end(1), p.y) == 0);
	mu_check(fltcmp(quad->carrot_controller->wp_end(2), quad->hover_point->z) == 0);

	mu_check(quad->carrot_controller->waypoints.size() == 5);
	mu_check(quad->carrot_controller->initialized == 1);

    return 0;
}

int testQuadrotorRunCarrotMode(void)
{
    Quadrotor *quad;
    Pose p;
    Pose robot_pose;
    Position cmd;
    float x_waypoints[5] = {0.0, 5.0, 5.0, 0.0, 0.0};
    float y_waypoints[5] = {0.0, 0.0, 5.0, 5.0, 0.0};
    float dt;

    // setup
	quad = testSetup();

	dt = 0.1;

	p.x = 0.0;
	p.y = 0.0;
	p.z = 3.0;
	p.roll = 0.0;
	p.pitch = 0.0;
	p.yaw = 0.0;

	quad->updatePose(p);
	quad->initializeCarrotController();

	// test and assert
	for (int i = 0; i < 5; i++) {
        // (i + 1)-th waypoint
        robot_pose.x = x_waypoints[i];
        robot_pose.y = y_waypoints[i];
        robot_pose.z = quad->hover_point->z;
        cmd = quad->runCarrotMode(robot_pose, dt);

        if (i < 3) {
            // check waypoint 2 to 4
            mu_check(quad->carrot_controller->waypoints.size() == (5 - i));
        } else {
            // check waypoint 5
            mu_check(quad->carrot_controller->waypoints.size() == 2);
        }

        mu_print("pos: %f %f %f\n", robot_pose.x, robot_pose.y, robot_pose.z);
        mu_print("cmd: %f %f %f\n", cmd.x, cmd.y, cmd.z);
        mu_print("waypoints: %d\n\n", (int) quad->carrot_controller->waypoints.size());
	}

    // check if hover mode activated
	mu_check(quad->mission_state == HOVER_MODE);
	mu_check(quad->hover_point->initialized == true);
	mu_check(fltcmp(quad->hover_point->x, quad->carrot_controller->wp_end(0)) == 0);
	mu_check(fltcmp(quad->hover_point->y, quad->carrot_controller->wp_end(1)) == 0);
	mu_check(fltcmp(quad->hover_point->z, quad->carrot_controller->wp_end(2)) == 0);

    return 0;
}

int testQuadrotorRunDiscoveryMode(void)
{
    Quadrotor *quad;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    Position cmd;
    float dt;

    // setup
	quad = testSetup();
	dt = 0.1;

	// test transition to tracking mode
	robot_pose.x = 1.0;
	robot_pose.y = 2.0;
	robot_pose.z = 3.0;

	landing_zone.detected = true;
	landing_zone.x = 0.0;
	landing_zone.y = 0.0;
	landing_zone.z = 0.0;

    cmd = quad->runDiscoverMode(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == TRACKING_MODE);

    // test hover during discover mode
	landing_zone.detected = false;
    cmd = quad->runDiscoverMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(cmd.x, 1.0) == 0);
    mu_check(fltcmp(cmd.y, 2.0) == 0);
    mu_check(fltcmp(cmd.z, quad->hover_point->z) == 0);

    // check quad->tracking_start was instanciated within 2 seconds ago
    mu_check(quad->tracking_start <= time(NULL));
    mu_check(quad->tracking_start >= time(NULL) - 2);

    return 0;
}

int testQuadrotorRunTrackingMode(void)
{
    Quadrotor *quad;
    float dt;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    Position cmd;

    // setup
	quad = testSetup();
	dt = 0.1;

	// test tracking mode - time step 1
	robot_pose.x = 1.0;
	robot_pose.y = 2.0;
	robot_pose.z = 3.0;

	landing_zone.detected = true;
	landing_zone.x = 0.0;
	landing_zone.y = 0.0;
	landing_zone.z = 0.0;

	quad->runDiscoverMode(robot_pose, landing_zone, dt);
	cmd = quad->runTrackingMode(robot_pose, landing_zone, dt);
    mu_print("cmd: %f %f %f\n\n", cmd.x, cmd.y, cmd.z);

	// test tracking mode - time step 2
	robot_pose.x = 1.1;
	robot_pose.y = 2.0;
	robot_pose.z = 3.0;

	landing_zone.x = 0.0;
	landing_zone.y = 0.0;
	landing_zone.z = 0.0;
	cmd = quad->runTrackingMode(robot_pose, landing_zone, dt);
    mu_print("cmd: %f %f %f\n\n", cmd.x, cmd.y, cmd.z);

	// test tracking mode - time step 3
	robot_pose.x = 1.2;
	robot_pose.y = 2.0;
	robot_pose.z = 3.0;

	landing_zone.x = 0.0;
	landing_zone.y = 0.0;
	landing_zone.z = 0.0;
	cmd = quad->runTrackingMode(robot_pose, landing_zone, dt);
    mu_print("cmd: %f %f %f\n\n", cmd.x, cmd.y, cmd.z);

    return 0;
}

int testQuadrotorRunLandingMode(void)
{
    Quadrotor *quad;
    float dt;
    float old_hover_height;
    Pose robot_pose;
    LandingTargetPosition landing_zone;
    Position cmd;

    // setup
	quad = testSetup();
	dt = 0.1;
	old_hover_height = quad->hover_point->z;

	robot_pose.x = 0.0;
	robot_pose.y = 0.0;
	robot_pose.z = 3.0;

	landing_zone.detected = true;
	landing_zone.x = 0.0;
	landing_zone.y = 0.0;
	landing_zone.z = 4.0;

	quad->runDiscoverMode(robot_pose, landing_zone, dt);
	quad->mission_state = LANDING_MODE;

	// test landing mode - lower height
    quad->height_last_updated = time(NULL) - 2;
	cmd = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(quad->hover_point->z < old_hover_height);

    // test landing mode - do not lower height
    old_hover_height = quad->hover_point->z;
    quad->height_last_updated = time(NULL);
    landing_zone.x = 1;
    landing_zone.y = 1;
	cmd = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(quad->hover_point->z, old_hover_height) == 0);

	// test landing mode - increase height
    quad->height_last_updated = time(NULL) - 2;
    landing_zone.x = 1;
    landing_zone.y = 1;
	cmd = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(quad->hover_point->z > old_hover_height);

    // test landing mode - do not increase height
    old_hover_height = quad->hover_point->z;
    quad->height_last_updated = time(NULL);
    landing_zone.x = 1;
    landing_zone.y = 1;
	cmd = quad->runLandingMode(robot_pose, landing_zone, dt);
    mu_check(fltcmp(quad->hover_point->z, old_hover_height) == 0);

    // test landing mode - kill engines
    landing_zone.x = 0.19;
    landing_zone.y = 0.19;
    landing_zone.z = 0.39;
	cmd = quad->runLandingMode(robot_pose, landing_zone, dt);
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

    // test IDLE_MODE
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == DISCOVER_MODE);
    mu_check(quad->hover_point->initialized ==  true);
    mu_check(fltcmp(quad->hover_point->x, robot_pose.x) == 0);
    mu_check(fltcmp(quad->hover_point->y, robot_pose.y) == 0);
    mu_check(fltcmp(quad->hover_point->z, 3) == 0);

    // test TRACKER_MODE
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == TRACKING_MODE);
    mu_check(fltcmp(quad->hover_point->x, robot_pose.x) == 0);
    mu_check(fltcmp(quad->hover_point->y, robot_pose.y) == 0);
    mu_check(fltcmp(quad->hover_point->z, robot_pose.z + 3) == 0);

    quad->tracking_start = quad->tracking_start - 6;  // emulate tracking for 6 seconds
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == LANDING_MODE);

    // test LANDING_MODE
    landing_zone.x = 0.19;
    landing_zone.y = 0.19;
    landing_zone.z = 0.39;  // set detected landing zone within disarm range
	quad->runLandingMode(robot_pose, landing_zone, dt);
	mu_check(quad->mission_state == MISSION_ACCOMPLISHED);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testQuadrotor);
    mu_add_test(testQuadrotorUpdatePose);
    mu_add_test(testQuadrotorPositionControllerCalculate);
    mu_add_test(testQuadrotorResetPositionController);
    mu_add_test(testQuadrotorRunIdleMode);
    mu_add_test(testQuadrotorRunHoverMode);
    mu_add_test(testQuadrotorInitializeCarrotController);
    mu_add_test(testQuadrotorRunCarrotMode);
    mu_add_test(testQuadrotorRunDiscoveryMode);
    mu_add_test(testQuadrotorRunTrackingMode);
    mu_add_test(testQuadrotorRunLandingMode);
    mu_add_test(testQuadrotorRunMission);
}

mu_run_tests(testSuite)
