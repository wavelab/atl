#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include "awesomo/util.hpp"
#include "awesomo/controller.hpp"


// STATES
#define IDLE_MODE 0
#define INITIALIZE_MODE 1
#define CARROT_MODE 2
#define TRACKING_MODE 3
#define LAND_MODE 4

class Quadrotor
{
public:
    // state
    int mission_state;
    Pose pose;
    Position landing_zone_prev;
    Position landing_zone_world;

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
