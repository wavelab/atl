#ifndef __RC_HPP__
#define __RC_HPP__

#include <stdio.h>

#include "navio2/RCInput.h"


class RCControl
{
    public:
        RCInput *rc_control;
        int channel_1;
        int channel_2;
        int channel_3;
        int channel_4;
        int channel_5;
        int channel_6;

        RCControl(void);
        void update(void);
        void print(void);
};


#endif
