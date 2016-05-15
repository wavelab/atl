#include "fcu.hpp"


FCU::FCU(void)
{
    this->state = FCU_IDLE;

    this->imu = new IMU();
    this->motors = new Motors();
    this->rc_control = new RCControl();

    this->attitude_controller = new AttitudeController();
}

void FCU::intializeAttitudeEstimator(void)
{
    this->attitude_estimator;

}

int FCU::initialize(void)
{
    this->imu->initialize();
    this->motors->arm();

    this->state = FCU_ARMED;

    return 0;
}

int runRCMode(void)
{
    float roll;
    float pitch;
    float yaw;
    float throttle;

    // TODO: need to extract out magic numbers and load radio calibration
    roll = (this->rc_control->ch1 - 1500.0) / (2000.0 - 1500.0);
    pitch = (this->rc_control->ch2 - 1500.0) / (2000.0 - 1500.0);
    throttle = (this->rc_control->ch3 - 1092) / (1900.0 - 1092.0);

    this->motors->set_throttle(0, throttle - roll + pitch);
    this->motors->set_throttle(1, throttle + roll - pitch);
    this->motors->set_throttle(2, throttle + roll + pitch);
    this->motors->set_throttle(3, throttle - roll - pitch);

    // switch to autopilot mode
    if (this->rc_control->ch5 > 1500) {
        this->state = FCU_AUTOPILOT;
    }

    return 0;
}

int runAutopilotMode(void)
{
    Orientation orientation;
    float throttle;

    // setup
    orientation.roll = imu.roll;
    orientation.pitch = imu.pitch;
    throttle = 0.5;

    // calculate attitude
    this->attitude_controller->calculate(orientation, throttle);

    // set motor speeds
    this->motors->set_throttle(0, this->attitude_controller->m1);
    this->motors->set_throttle(1, this->attitude_controller->m2);
    this->motors->set_throttle(2, this->attitude_controller->m3);
    this->motors->set_throttle(3, this->attitude_controller->m4);

    // switch to RC mode
    if (this->rc_control->ch5 < 1500) {
        this->state = FCU_RC_CONTROL;
    }

    return 0;
}

int FCU::run(void)
{
    while (1) {
        this->rc_control->update();

        if (this->state == FCU_RC_CONTROL) {
            this->runRCMode();
        } else if (this->state == FCU_AUTOPILOT) {
            this->runAutopilotMode();
        }
    }

    return 0;
}
