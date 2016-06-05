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
#define KF_DISCOVER_MODE 4
#define KF_TRACKING_MODE 5
#define KF_LANDING_MODE 6


class Quadrotor
{
public:
    // state
    int mission_state;

    Pose pose;
    Position going_to;

    float hover_height;
    int landing_zone_belief;

    time_t tracking_start;
    time_t height_last_updated;

    // controllers
    CarrotController *carrot_controller;
    PositionController *position_controller;

    // estimators
    struct kf apriltag_estimator;

    Quadrotor(std::map<std::string, std::string> configs);
    Attitude positionControllerCalculate(Position p, float dt);
    void updatePose(Pose p);
    void resetPositionController(void);
    void initializeMission(void);
    int runMission(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
};

#endif
