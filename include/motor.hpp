#ifndef __MOTOR_HPP__
#define __MOTOR_HPP__


#include <unistd.h>

#include "navio2/PWM.h"


// CONSTANTS
#define DUTY_MIN 1.00  // milliseconds
#define DUTY_MAX 2.00  // milliseconds


// CLASSES
class Motors
{
    public:
        PWM *pwm;
        float period;
        float motor_1;
        float motor_2;
        float motor_3;
        float motor_4;

        Motors(void);
        void set_throttle(int index, float percentage);
        void set_throttle_pwm(int index, int pwm);
};

#endif
