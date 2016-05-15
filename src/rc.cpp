#include "rc.hpp"


RCControl::RCControl(void)
{
    this->rc_control = new RCInput();
    this->calibrated = false;

    ch1 = 0;
    ch1_min = 0;
    ch1_max = 0;

    ch2 = 0;
    ch2_min = 0;
    ch2_max = 0;

    ch3 = 0;
    ch3_min = 0;
    ch3_max = 0;

    ch4 = 0;
    ch4_min = 0;
    ch4_max = 0;

    ch5 = 0;
    ch5_min = 0;
    ch5_max = 0;

    ch6 = 0;
    ch6_min = 0;
    ch6_max = 0;
}

RCControl::RCControl(std::string config_path)
{
    YAML::Node config;

    this->rc_control = new RCInput();
    this->calibrated = true;

    config = YAML::LoadFile(config_path);

    ch1 = 0;
    ch1_min = config["channel_1"]["min"].as<int>();
    ch1_max = config["channel_1"]["max"].as<int>();

    ch2 = 0;
    ch2_min = config["channel_2"]["min"].as<int>();
    ch2_max = config["channel_2"]["max"].as<int>();

    ch3 = 0;
    ch3_min = config["channel_3"]["min"].as<int>();
    ch3_max = config["channel_3"]["max"].as<int>();

    ch4 = 0;
    ch4_min = config["channel_4"]["min"].as<int>();
    ch4_max = config["channel_4"]["max"].as<int>();

    ch5 = 0;
    ch5_min = config["channel_5"]["min"].as<int>();
    ch5_max = config["channel_5"]["max"].as<int>();

    ch6 = 0;
    ch6_min = config["channel_6"]["min"].as<int>();
    ch6_max = config["channel_6"]["max"].as<int>();
}

int RCControl::initialize(void)
{
    this->rc_control->init();
    return 0;
}

void RCControl::update(void)
{
    this->ch1 = this->rc_control->read(0);
    this->ch2 = this->rc_control->read(1);
    this->ch3 = this->rc_control->read(2);
    this->ch4 = this->rc_control->read(3);
    this->ch5 = this->rc_control->read(4);
    this->ch6 = this->rc_control->read(5);
}

void RCControl::print(void)
{
    printf("channel_1: %d\n", this->ch1);
    printf("channel_2: %d\n", this->ch2);
    printf("channel_3: %d\n", this->ch3);
    printf("channel_4: %d\n", this->ch4);
    printf("channel_5: %d\n", this->ch5);
    printf("channel_6: %d\n", this->ch6);
}
