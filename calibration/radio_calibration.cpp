#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#include <yaml-cpp/yaml.h>

#include "rc.hpp"

#define NONBLOCK_ENABLE 0
#define NONBLOCK_DISABLE 1

#define RADIO_CALIBRATION_INSTRUCTIONS \
    "Start Radio Calibration\n" \
    "Move all radio channels to its extreme positions!\n" \
    "Once you are done press the ENTER key to exit!\n" \
    "You may also press 'p' to print current channel bounds\n";


struct channel_bounds
{
    int ch1_min;
    int ch1_max;

    int ch2_min;
    int ch2_max;

    int ch3_min;
    int ch3_max;

    int ch4_min;
    int ch4_max;

    int ch5_min;
    int ch5_max;

    int ch6_min;
    int ch6_max;
};

int kbhit()
{
    struct timeval tv;

    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);  // STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);

    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state)
{
    struct termios ttystate;

    // get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);

    if (state == NONBLOCK_ENABLE) {
        ttystate.c_lflag &= ~ICANON;  // turn off canonical mode
        ttystate.c_cc[VMIN] = 1;  // minimum of number input read.
    } else if (state == NONBLOCK_DISABLE) {
        ttystate.c_lflag |= ICANON;  // turn on canonical mode
    }

    // set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

void initialize_channel_bounds(struct channel_bounds *cb, RCControl &rc)
{
    rc.update();

    cb->ch1_min = rc.ch1;
    cb->ch1_max = rc.ch1;

    cb->ch2_min = rc.ch2;
    cb->ch2_max = rc.ch2;

    cb->ch3_min = rc.ch3;
    cb->ch3_max = rc.ch3;

    cb->ch4_min = rc.ch4;
    cb->ch4_max = rc.ch4;

    cb->ch5_min = rc.ch5;
    cb->ch5_max = rc.ch5;

    cb->ch6_min = rc.ch6;
    cb->ch6_max = rc.ch6;
}

void record_channel_bounds(struct channel_bounds *cb, RCControl &rc)
{
    cb->ch1_min = std::min(rc.ch1, cb->ch1_min);
    cb->ch1_max = std::max(rc.ch1, cb->ch1_max);

    cb->ch2_min = std::min(rc.ch2, cb->ch2_min);
    cb->ch2_max = std::max(rc.ch2, cb->ch2_max);

    cb->ch3_min = std::min(rc.ch3, cb->ch3_min);
    cb->ch3_max = std::max(rc.ch3, cb->ch3_max);

    cb->ch4_min = std::min(rc.ch4, cb->ch4_min);
    cb->ch4_max = std::max(rc.ch4, cb->ch4_max);

    cb->ch5_min = std::min(rc.ch5, cb->ch5_min);
    cb->ch5_max = std::max(rc.ch5, cb->ch5_max);

    cb->ch6_min = std::min(rc.ch6, cb->ch6_min);
    cb->ch6_max = std::max(rc.ch6, cb->ch6_max);
}

void print_channel_bounds(struct channel_bounds *cb)
{
    std::cout << std::endl;

    std::cout << "[Channel 1]";
    std::cout << "\t min: " << cb->ch1_min;
    std::cout << "\t max: " << cb->ch1_max;
    std::cout << std::endl;

    std::cout << "[Channel 2]";
    std::cout << "\t min: " << cb->ch2_min;
    std::cout << "\t max: " << cb->ch2_max;
    std::cout << std::endl;

    std::cout << "[Channel 3]";
    std::cout << "\t min: " << cb->ch3_min;
    std::cout << "\t max: " << cb->ch3_max;
    std::cout << std::endl;

    std::cout << "[Channel 4]";
    std::cout << "\t min: " << cb->ch4_min;
    std::cout << "\t max: " << cb->ch4_max;
    std::cout << std::endl;

    std::cout << "[Channel 5]";
    std::cout << "\t min: " << cb->ch5_min;
    std::cout << "\t max: " << cb->ch5_max;
    std::cout << std::endl;

    std::cout << "[Channel 6]";
    std::cout << "\t min: " << cb->ch6_min;
    std::cout << "\t max: " << cb->ch6_max;
    std::cout << std::endl;

    std::cout << std::endl;
}

int output_channel_bounds(struct channel_bounds *cb)
{
    std::ofstream outfile;
    YAML::Emitter yaml;

    // setup
    std::cout << "Saving radio calibration to [radio.yaml]" << std::endl;
    outfile.open("radio.yaml");

    // channel 1
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "channel_1";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "min";
        yaml << YAML::Value << cb->ch1_min;
        yaml << YAML::Key << "max";
        yaml << YAML::Value << cb->ch1_max;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // channel 2
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "channel_2";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "min";
        yaml << YAML::Value << cb->ch2_min;
        yaml << YAML::Key << "max";
        yaml << YAML::Value << cb->ch2_max;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // channel 3
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "channel_3";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "min";
        yaml << YAML::Value << cb->ch3_min;
        yaml << YAML::Key << "max";
        yaml << YAML::Value << cb->ch3_max;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // channel 4
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "channel_4";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "min";
        yaml << YAML::Value << cb->ch4_min;
        yaml << YAML::Key << "max";
        yaml << YAML::Value << cb->ch4_max;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // channel 5
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "channel_5";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "min";
        yaml << YAML::Value << cb->ch5_min;
        yaml << YAML::Key << "max";
        yaml << YAML::Value << cb->ch5_max;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // channel 6
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "channel_6";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "min";
        yaml << YAML::Value << cb->ch6_min;
        yaml << YAML::Key << "max";
        yaml << YAML::Value << cb->ch6_max;
        yaml << YAML::EndMap;
    yaml << YAML::EndMap;

    // write to file
    outfile << yaml.c_str() << std::endl;
    outfile.close();
    std::cout << "Done!" << std::endl;

    return 0;
}

int main(void)
{
    int loop;
    char c;
    RCControl rc;
    struct channel_bounds cb;

    // setup
    loop = 1;
    nonblock(NONBLOCK_ENABLE);
    initialize_channel_bounds(&cb, rc);
    std::cout << RADIO_CALIBRATION_INSTRUCTIONS;

    // calibrate radio
    while (loop) {
        // get channel bounds
        rc.update();
        record_channel_bounds(&cb, rc);

        // detect keyboard event
        if (kbhit()) {
            c = fgetc(stdin);

            if (c == 10) {
                // exit if ENTER key was pressed
                loop = 0;
                output_channel_bounds(&cb);
            } else if (c == 'p') {
                // print current channel bounds
                print_channel_bounds(&cb);
            }
        }
    }

    return 0;
}
