#ifndef __RC_HPP__
#define __RC_HPP__

#include <stdio.h>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include "navio2/RCInput.h"


class RCControl
{
public:
    RCInput *rc_control;

    bool calibrated;

    int ch1;
    int ch1_min;
    int ch1_max;

    int ch2;
    int ch2_min;
    int ch2_max;

    int ch3;
    int ch3_min;
    int ch3_max;

    int ch4;
    int ch4_min;
    int ch4_max;

    int ch5;
    int ch5_min;
    int ch5_max;

    int ch6;
    int ch6_min;
    int ch6_max;

    RCControl(void);
    RCControl(std::string config_path);
    void update(void);
    void print(void);
};


#endif
