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
    // track time
    float min_track_time;
    float target_lost_limit;

    // position offsets
    float offset_x;
    float offset_y;
    float offset_z;

    // constructor
    TrackingConfig(void):
        offset_x(0),
        offset_y(0),
        offset_z(0),
        min_track_time(10) {}
};

class LandingConfig
{
public:
    // height update
    float period;
    float descend_multiplier;
    float recover_multiplier;
    float x_threshold;
    float y_threshold;

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
        x_threshold(0),
        y_threshold(0),
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
    time_t tracking_start;
    time_t tag_last_updated;
    int landing_zone_belief;
    time_t height_last_updated;

    // controllers
    CarrotController *carrot_controller;
    PositionController *position_controller;

    // estimators
    bool estimator_initialized;
    struct kf tag_estimator;

    // constructors
    Quadrotor(std::map<std::string, std::string> configs);

    // methods
    int loadConfig(std::string config_file_path);
    void updatePose(Pose p);
    void updateHoverPointWithTag(Pose robot_pose, float tag_x, float tag_y);
    bool hasLanded(LandingTargetPosition landing_zone);
    Attitude positionControllerCalculate(
        Position setpoint,
        Pose robot_pose,
        float dt
    );
    void resetPositionController(void);
    void runIdleMode(Pose robot_pose);
    Position runHoverMode(Pose robot_pose, float dt);
    void initializeCarrotController(void);
    Position runCarrotMode(Pose robot_pose, float dt);
    Position runDiscoverMode(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
    Position trackApriltag(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
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
