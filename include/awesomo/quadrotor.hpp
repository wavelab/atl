#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include "awesomo/util.hpp"
#include "awesomo/controller.hpp"


// CONSTANTS
#define LZ_THRESHOLD 5

// STATES
#define IDLE_MODE 0
#define DISCOVER_MODE 1
#define CARROT_INITIALIZE_MODE 2
#define CARROT_MODE 3
#define TRACKING_MODE 4
#define CARROT_TRACKER_MODE 5


class Quadrotor
{
public:
    // state
    int mission_state;

    Pose pose;
    float hover_height;
    int landing_zone_belief;
    Position going_to;
    time_t wp_last_added;

    // controllers
    CarrotController *carrot_controller;
    PositionController *position_controller;

    Quadrotor(std::map<std::string, std::string> configs);
    Attitude positionControllerCalculate(Position p, float dt);
    void updatePose(Pose p);
    void resetPositionController(void);
    void initializeMission(void);
    void runMission(
        Pose robot_pose,
        LandingTargetPosition landing_zone,
        float dt
    );
};

#endif
