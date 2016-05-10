#include "motor.hpp"


Motors::Motors(void)
{
    this->pwm = new PWM();
    this->period = 200;
}

void Motors::arm(void)
{
    this->pwm->init(0);
    this->pwm->init(1);
    this->pwm->init(2);
    this->pwm->init(3);

    this->pwm->enable(0);
    this->pwm->enable(1);
    this->pwm->enable(2);
    this->pwm->enable(3);

    this->pwm->set_period(0, this->period);
    this->pwm->set_period(1, this->period);
    this->pwm->set_period(2, this->period);
    this->pwm->set_period(3, this->period);

    this->motor_1 = 0.0;
    this->motor_2 = 0.0;
    this->motor_3 = 0.0;
    this->motor_4 = 0.0;

    sleep(2);
}

void Motors::set_throttle(int index, float percentage)
{
    float duty_cycle;
    float duty_range;

    // calculate duty cycle
    duty_range = DUTY_MAX - DUTY_MIN;
    duty_cycle = DUTY_MIN + (duty_range * percentage);

    // bound the duty cycle
    if (duty_cycle > DUTY_MAX) {
        duty_cycle = DUTY_MAX;
    } else if (duty_cycle < DUTY_MIN) {
        duty_cycle = DUTY_MIN;
    }

    this->pwm->set_duty_cycle(index, duty_cycle);
}

void Motors::set_throttle_pwm(int index, int pwm)
{
    this->pwm->set_duty_cycle(index, pwm);
}
