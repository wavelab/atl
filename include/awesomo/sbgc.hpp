#ifndef __SBGC_HPP__
#define __SBGC_HPP__

#include <unistd.h>

#include <iostream>

#include <serial/serial.h>


#define FRAME_ANG_CONV 0.02197265625  // deg per bit

// GENERAL
#define SBGC_CMD_MAX_BYTES 255
#define SBGC_CMD_PAYLOAD_BYTES 5


// CMD ID
#define CMD_READ_PARAMS  82
#define CMD_WRITE_PARAMS  87
#define CMD_REALTIME_DATA  68
#define CMD_BOARD_INFO  86
#define CMD_CALIB_ACC  65
#define CMD_CALIB_GYRO  103
#define CMD_CALIB_EXT_GAIN  71
#define CMD_USE_DEFAULTS  70
#define CMD_CALIB_POLES  80
#define CMD_RESET  114
#define CMD_HELPER_DATA 72
#define CMD_CALIB_OFFSET  79
#define CMD_CALIB_BAT  66
#define CMD_MOTORS_ON   77
#define CMD_MOTORS_OFF  109
#define CMD_CONTROL   67
#define CMD_TRIGGER_PIN  84
#define CMD_EXECUTE_MENU 69
#define CMD_GET_ANGLES  73
#define CMD_CONFIRM  67


// CMD CONTROL
#define MODE_NO_CONTROL 0
#define MODE_SPEED 1
#define MODE_ANGLE 2
#define MODE_SPEED_ANGLE 3
#define MODE_RC 4
#define MODE_ANGLE_REL_FRAME 5


class SBGCCommand
{
public:
	// header
    uint8_t cmd_id;
    uint8_t data_size;
	uint8_t header_checksum;

	// body
    uint8_t *data;
    uint8_t data_checksum;
};

class SBGC
{
public:
    std::string port;
    unsigned long baudrate;
    serial::Timeout timeout;
    serial::Serial serial;

    SBGC(std::string port, unsigned long baudrate, int timeout);
    int init(void);
    int getBoardInfo(void);
    int sendCommand(SBGCCommand &cmd);
    int setAngle(double roll, double pitch, double yaw);
};

#endif
