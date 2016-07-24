#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include "awesomo/util.hpp"
#include "awesomo/controller.hpp"
#include "awesomo/estimator.hpp"


// CONSTANTS
#define LZ_THRESHOLD 0

// STATES
#define TARGET_LOST -2
#define MISSION_ACCOMPLISHED -1
#define IDLE_MODE 0
#define HOVER_MODE 1
#define CARROT_INITIALIZE_MODE 2
#define CARROT_MODE 3
#define DISCOVER_MODE 4
#define TRACKING_MODE 5
#define LANDING_MODE 6


// TIMEOUTS
#define TARGET_LOST_TIMEOUT 2 // run on kalman estimate until this time
#define LANDING_DELAY 5  // observe the target before landing

class HoverPoint
{
public:
    // hover point properties
    bool initialized;
    Eigen::Vector3d position;
    float hover_height;

    // constructors
    HoverPoint();
    HoverPoint(float x, float y, float z);
};


class LandingConfig
{
public:
    // height update
    float period;
    float descend_multiplier;
    float recover_multiplier;

    // disarm conditions
    float belief_threshold;
    Eigen::Vector3d cutoff_position;

    // constructor
    LandingConfig(void);
    LandingConfig(
        float period,
        float desend_multiplier,
        float recover_multiplier,
        float belief_threshold,
        Eigen::Vector3d cutoff_position
);

};

class Quadrotor
{
public:
    // configs
    LandingConfig *landing_config;

    // state
    int mission_state;
    Pose global_pose;

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
        Eigen::Vector3d setpoint,
        Pose robot_pose,
        float yaw,
        float dt
    );

    // void updatePose(Pose p);
    void resetPositionController(void);
    void runIdleMode(Pose robot_pose);
    Eigen::Vector3d runHoverMode(Pose robot_pose, float dt);
    void initializeCarrotController(void);
    Eigen::Vector3d runCarrotMode(Pose robot_pose, float dt);
    Eigen::Vector3d runDiscoverMode(
        Pose robot_pose,
        LandingTargetPosition landing_zone
    );
    Eigen::Vector3d runTrackingModeBPF(
        LandingTargetPosition landing_zone,
        float dt
    );
    Eigen::Vector3d runLandingMode(
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
