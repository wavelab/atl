#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include "awesomo/util.hpp"
#include "awesomo/controller.hpp"
#include "awesomo/estimator.hpp"


// CONSTANTS
#define LZ_THRESHOLD 0

// STATES
#define MISSION_ACCOMPLISHED -1
#define IDLE_MODE 0
#define HOVER_MODE 1
#define CARROT_INITIALIZE_MODE 2
#define CARROT_MODE 3
#define DISCOVER_MODE 4
#define TRACKING_MODE 5
#define LANDING_MODE 6


class HoverPoint
{
public:
    // hover point properties
    bool initialized;
    float x;
    float y;
    float z;

    // constructor
    HoverPoint(void):
        initialized(false),
        x(0),
        y(0),
        z(0) {}
};

class TrackingConfig
{
public:
    // position offsets
    float offset_x;
    float offset_y;
    float offset_z;

    // constructor
    TrackingConfig(void):
        offset_x(0),
        offset_y(0),
        offset_z(0) {}
};

class LandingConfig
{
public:
    // height update
    float period;
    float descend_multiplier;
    float recover_multiplier;

    // disarm conditions
    float x_cutoff;
    float y_cutoff;
    float z_cutoff;
    int belief_threshold;

    // constructor
    LandingConfig(void):
        period(0),
        descend_multiplier(0),
        recover_multiplier(0),
        x_cutoff(0),
        y_cutoff(0),
        z_cutoff(0),
        belief_threshold(0) {}
};

class Quadrotor
{
public:
    // configs
    TrackingConfig *tracking_config;
    LandingConfig *landing_config;

    // state
    int mission_state;
    Pose pose;
    HoverPoint *hover_point;
    int landing_zone_belief;
    time_t tracking_start;
    time_t target_last_updated;
    time_t height_last_updated;

    // controllers
    CarrotController *carrot_controller;
    PositionController *position_controller;

    // estimators
    bool estimator_initialized;
    struct kf apriltag_estimator;

    Quadrotor(std::map<std::string, std::string> configs);
    int loadConfig(std::string config_file_path);
    Attitude positionControllerCalculate(
        Position setpoint,
        Pose robot_pose,
        float dt
    );
    void updatePose(Pose p);
    void resetPositionController(void);
    void runIdleMode(Pose robot_pose);
    Position runHoverMode(Pose robot_pose, float dt);
    void initializeCarrotController(void);
    Position runCarrotMode(Pose robot_pose, float dt);
    Position runDiscoverMode(
        Pose robot_pose,
        LandingTargetPosition landing_zone
    );
    Position runTrackingMode(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
    Position runLandingMode(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
    int followWaypoints(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
    int runMission(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
};

#endif
