#include "awesomo/sbgc.hpp"


SBGC::SBGC(std::string port, unsigned long baudrate, int timeout)
{
    this->port = port;
    this->baudrate = baudrate;
    this->timeout = serial::Timeout::simpleTimeout(timeout);
    this->serial.setTimeout(this->timeout);
    this->serial.setPort(this->port);
    this->serial.setBaudrate(this->baudrate);
}

int SBGC::init(void)
{
	this->serial.open();

    if (this->serial.isOpen()) {
        std::cout << "Yes!" << std::endl;
    } else {
        std::cout << "No!" << std::endl;
        return -1;
    }

    return 0;
}

int SBGC::getBoardInfo(void)
{


    return 0;
}

int SBGC::sendCommand(SBGCCommand &cmd)
{
    uint8_t start = 0x3E;  // ">" character

    // pre-check
    if (cmd.size >= (SBGC_CMD_MAX_BYTES - SBGC_CMD_PAYLOAD_BYTES)) {
        return -1;
    }

    // header
    this->serial.write(&start, 1);

    // command
    this->serial.write(&cmd.cmd_id, 1);

    // size
    this->serial.write(&cmd.data_size, 1);

    // header checksum
    this->serial.write(&cmd.header_checksum, 1);

    // data
    this->serial.write(cmd.data, (size_t) cmd.data_size);

    // data checksum
    this->serial.write(&cmd.data_checksum, 1);

    return 0;
}

int SBGC::setAngle(double roll, double pitch, double yaw)
{
	SBGCCommand cmd;
	int16_t roll_adjusted;
	int16_t pitch_adjusted;
	int16_t yaw_adjusted;

	// adjust roll, pitch and yaw
	roll_adjusted = roll / FRAME_ANG_CONV;
	pitch_adjusted = pitch / FRAME_ANG_CONV;
	yaw_adjusted = yaw / FRAME_ANG_CONV;

	// build header
	cmd.cmd_id = CMD_CONTROL;
	cmd.data_size = 13;
	cmd.header_checksum = (cmd.cmd_id + cmd.data_size) % 256;

	// build body
	cmd.data = (uint8_t *) malloc(sizeof(uint8_t) * 13);
	cmd.data[0] = MODE_ANGLE;

	// speed roll
	cmd.data[1] = 0;
	cmd.data[2] = 0;

	// angle roll
	cmd.data[3] = ((roll_adjusted >> 8) & 0xff);
	cmd.data[4] = ((roll_adjusted >> 0) & 0xff);

	// speed pitch
	cmd.data[5] = 0;
	cmd.data[6] = 0;

	// angle pitch
	cmd.data[7] = ((pitch_adjusted >> 8) & 0xff);
	cmd.data[8] = ((pitch_adjusted >> 0) & 0xff);

	// speed yaw
	cmd.data[9] = 0;
	cmd.data[10] = 0;

	// angle yaw
	cmd.data[11] = ((pitch_adjusted >> 8) & 0xff);
	cmd.data[12] = ((yaw_adjusted >> 0) & 0xff);

	// data checksum
	cmd.data_checksum = 0x0;
	for (int i = 0; i < 13; i++) {
		cmd.data_checksum += cmd.data[i];
	}
	cmd.data_checksum = cmd.data_checksum % 256;

	// send command
	this->serial.flushInput();
	this->sendCommand(cmd);
	usleep(30000);
	this->serial.flushInput();

    return 0;
}
