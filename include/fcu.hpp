#ifndef __FCU_HPP__
#define __FCU_HPP__

#include "rc.hpp"
#include "ekf.hpp"
#include "imu.hpp"
#include "motor.hpp"
#include "controller.hpp"


// CONTANTS
#define FCU_IDLE 0
#define FCU_ARMED 1
#define FCU_RC_CONTROL 2
#define FCU_AUTOPILOT 3


// CLASSES
class FCU
{
public:
    int state;

    IMU *imu;
    Motors *motors;
    RCControl *rc_control;

    struct ekf *attitude_estimator;
    AttitudeController *attitude_controller;

    struct ekf *position_estimator;
    PositionController *position_controller;

    FCU(void);
    int initialize(void);
    int runRCMode(void);
    int runAutopilotMode(void);
    int run(void);
};


#endif
