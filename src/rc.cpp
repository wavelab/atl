#include "rc.hpp"


RCControl::RCControl(void)
{
    this->rc_control = new RCInput();
    this->rc_control->init();
}

void RCControl::update(void)
{
    this->channel_1 = this->rc_control->read(1);
    this->channel_2 = this->rc_control->read(2);
    this->channel_3 = this->rc_control->read(3);
    this->channel_4 = this->rc_control->read(4);
    this->channel_4 = this->rc_control->read(5);
    this->channel_4 = this->rc_control->read(6);
}

void RCControl::print(void)
{
    printf("channel_1: %d\n", this->channel_1);
    printf("channel_2: %d\n", this->channel_2);
    printf("channel_3: %d\n", this->channel_3);
    printf("channel_4: %d\n", this->channel_4);
    printf("channel_5: %d\n", this->channel_5);
    printf("channel_6: %d\n", this->channel_6);
}
