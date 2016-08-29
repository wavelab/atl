#include <iostream>

#include "awesomo/munit.h"
#include "awesomo/quadrotor.hpp"

// CONFIGS
#define QUADROTOR_CONFIG "configs/quadrotor/config.yaml"
#define POSITION_CONTROLLER_CONFIG "configs/position_controller/config.yaml"
#define CARROT_CONTROLLER_CONFIG "configs/carrot_controller/config.yaml"


// TESTS
Quadrotor *testSetup(void);
void transitionQuadrotorToTrackingMode(Quadrotor *quad);
void print_position_controller_adjustment(const char *title, Quadrotor *quad);
int testQuadrotor(void);
int testQuadrotorPositionControllerCalculate(void);
int testQuadrotorResetPositionController(void);
int testQuadrotorCalculateLandingTargetYaw(void);
// int testQuadrotorResetPositionController(void);
// int testQuadrotorInitializeCarrotController(void);
// int testQuadrotorRunCarrotMode(void);
// int testQuadrotorRunDiscoveryMode(void);
// int testQuadrotorRunTrackingModeBPF(void);
// int testQuadrotorWithinLandingZone(void);
// int testQuadrotorRunLandingMode(void);
// int testQuadrotorRunMission(void);


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

void transitionQuadrotorToTrackingMode(Quadrotor *quad)
{
    LandingTargetPosition landing_zone;

    landing_zone.detected = true;
    landing_zone.position(0) = 0.0;
    landing_zone.position(1) = 0.0;
    landing_zone.position(2) = 3.0;
    quad->runDiscoverMode(landing_zone);

    for (int i = 0; i < 5; i++) {
        landing_zone.detected = true;
        landing_zone.position(0) = 1.0;
        landing_zone.position(1) = 2.0;
        landing_zone.position(2) = 3.0;
        quad->runDiscoverMode(landing_zone);
    }
}

void print_position_controller_adjustment(const char *title, Quadrotor *quad)
{
    mu_print("%s\n", title);
    mu_print("roll: %f\n", quad->position_controller->roll);
    mu_print("pitch: %f\n", quad->position_controller->pitch);
    mu_print("throttle: %f\n", quad->position_controller->throttle);
    mu_print("\n");
}

int testQuadrotor(void)
{
    Quadrotor *quad;

    // setup
    quad = testSetup();


    // state
    mu_check(quad->mission_state == DISCOVER_MODE);
    mu_check(quad->world_pose.position(0) == 0);
    mu_check(quad->world_pose.position(1) == 0);
    mu_check(quad->world_pose.position(2) == 0);
    mu_check(fltcmp(quad->yaw, 0.0) == 0);

    // landing state
    mu_check(quad->landing_belief == 0);

    // estimators
    mu_check(quad->estimator_initialized == false);

    // position controller and carrot controller
    mu_check(quad->position_controller != NULL);
    mu_check(quad->carrot_controller != NULL);

    // quadrotor configrations
    mu_check(quad->hover_height > 0);
    mu_check(quad->landing_config->period > 0);
    mu_check(quad->landing_config->descend_multiplier > 0);
    mu_check(quad->landing_config->recover_multiplier > 0);
    mu_check(quad->landing_config->cutoff_position(0));
    mu_check(quad->landing_config->cutoff_position(1));
    mu_check(quad->landing_config->cutoff_position(2));
    mu_check(quad->landing_config->belief_threshold);

    return 0;
}

int testQuadrotorPositionControllerCalculate(void)
{
    Quadrotor *quad;
    Pose robot_pose;
    Eigen::Vector3d setpoint;
    float dt;
    float hover_throttle;

    // setup
    quad = testSetup();
    robot_pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    setpoint = Eigen::Vector3d(0.0, 0.0, 0.0);
    dt = 0.1;
    hover_throttle = quad->position_controller->hover_throttle;

    // test no correction
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(0.0, 0.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);
    mu_check(quad->position_controller->throttle > 0);
    print_position_controller_adjustment("Test going hover", quad);

    // test correction point to forward
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(10.0, 0.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) < 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going forward", quad);

    // test correction point to backwards
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(-10.0, 0.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) > 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going backwards", quad);

    // test correction point to left
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(0.0, -10.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) < 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going left", quad);

    // test correction point to right
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(0.0, 10.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) > 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going right", quad);

    // test correction point to left-forward
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(10.0, -10.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) < 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) < 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going left-forward", quad);

    // test correction point to right-forward
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(10.0, 10.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) > 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) < 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going right-forward", quad);

    // test correction point to left-backward
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(-10.0, -10.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) < 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) > 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going left-backward", quad);

    // test correction point to right-backward
    robot_pose.position << 0.0, 0.0, 5.0;
    setpoint = Eigen::Vector3d(-10.0, 10.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) > 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) > 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test going right-backward", quad);

    // test correction point to increase height
    robot_pose.position << 0.0, 0.0, 3.0;
    setpoint = Eigen::Vector3d(0.0, 0.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);
    mu_check(quad->position_controller->throttle > hover_throttle);
    print_position_controller_adjustment("Test increase height", quad);

    // test correction point to decrease height
    robot_pose.position << 0.0, 0.0, 10.0;
    setpoint = Eigen::Vector3d(0.0, 0.0, 5.0);

    quad->position_controller->reset();
    quad->positionControllerCalculate(setpoint, robot_pose, 0.0, dt);

    mu_check(fltcmp(quad->position_controller->roll, 0.0f) == 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0f) == 0);
    mu_check(quad->position_controller->throttle < hover_throttle);
    print_position_controller_adjustment("Test decrease height", quad);

    return 0;
}

int testQuadrotorResetPositionController(void)
{
    Quadrotor *quad;

    // setup
    quad = testSetup();

    quad->resetPositionController();
    mu_check(fltcmp(quad->position_controller->roll, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->pitch, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->throttle, 0.0) == 0);

    mu_check(fltcmp(quad->position_controller->x.output, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.prev_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.sum_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.p_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.i_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->x.d_error, 0.0) == 0);

    mu_check(fltcmp(quad->position_controller->y.output, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.prev_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.sum_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.p_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.i_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->y.d_error, 0.0) == 0);

    mu_check(fltcmp(quad->position_controller->T.output, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.prev_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.sum_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.p_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.i_error, 0.0) == 0);
    mu_check(fltcmp(quad->position_controller->T.d_error, 0.0) == 0);

    return 0;
}

int testQuadrotorCalculateLandingTargetYaw(void)
{
    Quadrotor *quad;
    double yaw;
    Eigen::Vector2d p;

    // setup
    quad = testSetup();

    // test - quadrant I
    p << 0, 0;
    quad->lt_history.push_back(p);
    p << 1, 1;
    quad->lt_history.push_back(p);

    quad->calculateLandingTargetYaw(&yaw);
    yaw = yaw * 180.0 / M_PI;
    mu_print("yaw (degrees): %f\n", yaw);
    quad->lt_history.clear();

    // test - quadrant II
    p << 0, 0;
    quad->lt_history.push_back(p);
    p << -1, 1;
    quad->lt_history.push_back(p);

    quad->calculateLandingTargetYaw(&yaw);
    yaw = yaw * 180.0 / M_PI;
    mu_print("yaw (degrees): %f\n", yaw);
    quad->lt_history.clear();

    // test - quadrant III
    p << 0, 0;
    quad->lt_history.push_back(p);
    p << -1, -1;
    quad->lt_history.push_back(p);

    quad->calculateLandingTargetYaw(&yaw);
    yaw = yaw * 180.0 / M_PI;
    mu_print("yaw (degrees): %f\n", yaw);
    quad->lt_history.clear();

    // test - quadrant IV
    p << 0, 0;
    quad->lt_history.push_back(p);
    p << 1, -1;
    quad->lt_history.push_back(p);

    quad->calculateLandingTargetYaw(&yaw);
    yaw = yaw * 180.0 / M_PI;
    mu_print("yaw (degrees): %f\n", yaw);
    quad->lt_history.clear();

    // test - over 360
    quad->yaw = 358.0 * M_PI / 180.0;
    p << 0, 0;
    quad->lt_history.push_back(p);
    p << 1, -1;
    quad->lt_history.push_back(p);

    quad->calculateLandingTargetYaw(&yaw);
    yaw = yaw * 180.0 / M_PI;
    mu_print("yaw (degrees): %f\n", yaw);
    quad->lt_history.clear();

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
//  quad = testSetup();
//
//  x = 0.0;
//  y = 0.0;
//  z = 3.0;
//  roll = 0.0;
//  pitch = 0.0;
//  yaw = 0.0;
//
//  p = Pose(roll, pitch, yaw, x, y, z);
//
//  quad->world_pose = p;
//  quad->initializeCarrotController();
//
//  // test and assert
//  for (int i = 0; i < 5; i++) {
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
//  }
//
//     // check if hover mode activated
//  mu_check(quad->mission_state == HOVER_MODE);
//  mu_check(fltcmp(quad->hover_point->position(0),
//      quad->carrot_controller->wp_end(0)) == 0);
//  mu_check(fltcmp(quad->hover_point->position(1),
//      quad->carrot_controller->wp_end(1)) == 0);
//  mu_check(fltcmp(quad->hover_height,
//      quad->carrot_controller->wp_end(2)) == 0);
//
//     return 0;
// }

int testQuadrotorRunDiscoveryMode(void)
{
    Quadrotor *quad;
    LandingTargetPosition landing_zone;

    // setup
    quad = testSetup();

    // test no transition away from discover mode
    landing_zone.detected = false;
    quad->runDiscoverMode(landing_zone);

    // test transition to tracking mode
    transitionQuadrotorToTrackingMode(quad);

    mu_check(quad->estimator_initialized == true);
    mu_check(fltcmp(quad->tag_estimator.mu(0), 1.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(1), 2.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(2), 3.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(3), 0.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(4), 0.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(5), 0.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(6), 0.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(7), 0.0) == 0);
    mu_check(fltcmp(quad->tag_estimator.mu(8), 0.0) == 0);
    mu_check(quad->mission_state == TRACKING_MODE);
    mu_check(quad->tracking_start <= time(NULL));
    mu_check(quad->tracking_start >= time(NULL) - 2);

    return 0;
}

int testQuadrotorRunTrackingModeBPF(void)
{
    Quadrotor *quad;
    float dt;
    Pose world_pose;
    LandingTargetPosition landing_zone;

    // setup
    quad = testSetup();
    dt = 0.1;

    // test tracking mode - time step 1
    landing_zone.detected = true;
    landing_zone.position(0) = 0.0;
    landing_zone.position(1) = 0.0;
    landing_zone.position(2) = 5.0;

    world_pose.position << 0.0, 0.0, 0.0;

    quad->world_pose = world_pose;

    // transition to tracking mode
    landing_zone.detected = true;
    landing_zone.position(0) = 0.0;
    landing_zone.position(1) = 0.0;
    landing_zone.position(2) = 3.0;
    quad->runDiscoverMode(landing_zone);

    for (int i = 0; i < 5; i++) {
        landing_zone.detected = true;
        landing_zone.position(0) = 1.0;
        landing_zone.position(1) = 2.0;
        landing_zone.position(2) = 3.0;
        quad->runDiscoverMode(landing_zone);
    }

    // test tracking
    quad->runTrackingModeBPF(landing_zone, dt);
    print_position_controller_adjustment("Gain altitude", quad);

    // test tracking mode - time step 2
    landing_zone.detected = true;
    landing_zone.position(0) = 1.0;
    landing_zone.position(1) = 0.0;
    landing_zone.position(2) = 5.0;

    world_pose.position << 0.0, 0.0, 5.0;

    quad->world_pose = world_pose;
    quad->runTrackingModeBPF(landing_zone, dt);
    print_position_controller_adjustment("Go forward", quad);

    // test tracking mode - time step 3
    landing_zone.detected = true;
    landing_zone.position(0) = -1.0;
    landing_zone.position(1) = 0.0;
    landing_zone.position(2) = 5.0;

    world_pose.position << 0.0, 0.0, 5.0;

    quad->world_pose = world_pose;
    quad->runTrackingModeBPF(landing_zone, dt);
    print_position_controller_adjustment("Go backwards", quad);

    return 0;
}

int testQuadrotorWithinLandingZone(void)
{
    Quadrotor *quad;
    Eigen::Vector3d tag_mea;
    Eigen::Vector3d tag_est;
    bool result;

    // setup
    quad = testSetup();

    // test fail
    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    tag_mea << 0.5, 0.5, 0.5;
    tag_est << 0.5, 0.5, 0.5;
    result = quad->withinLandingZone(tag_mea, tag_est);
    mu_check(result == false);

    // test measured pass
    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    tag_mea << 0.2, 0.2, 0.1;
    tag_est << 0.5, 0.5, 0.5;
    result = quad->withinLandingZone(tag_mea, tag_est);
    mu_check(result == true);

    // test estimated pass
    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    tag_mea << 0.5, 0.5, 0.5;
    tag_est << 0.2, 0.2, 0.1;
    result = quad->withinLandingZone(tag_mea, tag_est);
    mu_check(result == true);

    // test both pass
    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    tag_mea << 0.2, 0.2, 0.1;
    tag_est << 0.2, 0.2, 0.1;
    result = quad->withinLandingZone(tag_mea, tag_est);
    mu_check(result == true);

    // test overloaded function
    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    tag_mea << 0.2, 0.2, 0.1;
    result = quad->withinLandingZone(tag_mea);
    mu_check(result == true);

    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    tag_mea << 0.6, 0.6, 0.3;
    result = quad->withinLandingZone(tag_mea);
    mu_check(result == false);

    return 0;
}

int testQuadrotorRunLandingMode(void)
{
    Quadrotor *quad;
    float dt;
    Pose robot_pose;
    LandingTargetPosition landing_zone;

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

    quad->runDiscoverMode(landing_zone);
    transitionQuadrotorToTrackingMode(quad);

    quad->mission_state = LANDING_MODE;
    quad->landing_config->period = 2000.0;
    quad->landing_config->descend_multiplier = 0.8;
    quad->landing_config->recover_multiplier = 1.2;
    quad->landing_config->cutoff_position << 0.5, 0.5, 0.2;
    quad->landing_config->belief_threshold = 3;
    quad->hover_height= 6.0;

    // test - lower height
    tic(&quad->height_last_updated);
    quad->height_last_updated.tv_nsec -= (2000 * 1000000);
    quad->runLandingMode(landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.8) == 0);

    // test - do not lower height
    tic(&quad->height_last_updated);
    landing_zone.position(0) = 0.3;
    landing_zone.position(1) = 0.3;
    quad->runLandingMode(landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.8) == 0);

    // test - increase height
    tic(&quad->height_last_updated);
    quad->height_last_updated.tv_nsec -= (2000 * 1000000);
    landing_zone.position(0) = 0.51;
    landing_zone.position(1) = 0.51;
    quad->runLandingMode(landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.8 * 1.2) == 0);

    // test - do not increase height
    tic(&quad->height_last_updated);
    landing_zone.position(0) = 0.51;
    landing_zone.position(1) = 0.51;
    quad->runLandingMode(landing_zone, dt);
    mu_check(fltcmp(quad->hover_height, 4.8 * 1.2) == 0);

    // test - kill engines
    landing_zone.position << 0.19, 0.19, 0.19;
    quad->runLandingMode(landing_zone, dt);
    quad->runLandingMode(landing_zone, dt);
    quad->runLandingMode(landing_zone, dt);
    quad->runLandingMode(landing_zone, dt);
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
    mu_check(quad->mission_state == DISCOVER_MODE);
    quad->runMission(robot_pose, landing_zone, dt);
    transitionQuadrotorToTrackingMode(quad);
    mu_check(quad->mission_state == TRACKING_MODE);

    // test TRACKER_MODE
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == TRACKING_MODE);
    mu_check(quad->hover_height > 0);

    // emulate tracking for 11 seconds
    quad->tracking_start = quad->tracking_start - 11;
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == LANDING_MODE);

    // test LANDING_MODE
    // set detected landing zone within disarm range
    landing_zone.position << 0.19, 0.19, 0.19;
    quad->landing_config->belief_threshold = 3;
    quad->runMission(robot_pose, landing_zone, dt);
    quad->runMission(robot_pose, landing_zone, dt);
    quad->runMission(robot_pose, landing_zone, dt);
    quad->runMission(robot_pose, landing_zone, dt);
    mu_check(quad->mission_state == MISSION_ACCOMPLISHED);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testQuadrotor);
    mu_add_test(testQuadrotorPositionControllerCalculate);
    mu_add_test(testQuadrotorResetPositionController);
    mu_add_test(testQuadrotorCalculateLandingTargetYaw);
    // mu_add_test(testQuadrotorInitializeCarrotController);
    // mu_add_test(testQuadrotorRunCarrotMode);
    // mu_add_test(testQuadrotorRunDiscoveryMode);
    // mu_add_test(testQuadrotorRunTrackingModeBPF);
    // mu_add_test(testQuadrotorWithinLandingZone);
    mu_add_test(testQuadrotorRunLandingMode);
    // mu_add_test(testQuadrotorRunMission);
}

mu_run_tests(testSuite)
