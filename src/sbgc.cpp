#include "awesomo/sbgc.hpp"


void SBGCFrame::buildHeader(uint8_t cmd_id, uint8_t data_size)
{
	this->cmd_id = cmd_id;
	this->data_size = data_size;
	this->header_checksum = (this->cmd_id + this->data_size) % 256;
}

void SBGCFrame::buildDataChecksum(void)
{
    this->data_checksum = 0x0;
	for (int i = 0; i < this->data_size; i++) {
		this->data_checksum += this->data[i];
	}
	this->data_checksum = this->data_checksum % 256;
}

void SBGCFrame::buildBody(uint8_t *data)
{
	this->data = data;
	this->buildDataChecksum();
}

void SBGCFrame::buildCommand(int cmd_id, uint8_t *data, int data_size)
{
    this->buildHeader((uint8_t) cmd_id, (uint8_t) data_size);
    this->buildBody(data);
}

void SBGCFrame::buildCommand(int cmd_id)
{
    this->buildHeader((uint8_t) cmd_id, (uint8_t) 0);
}




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
        std::cout << "connected to sbgc!" << std::endl;
    } else {
        std::cout << "failed to connect to sbgc!" << std::endl;
        return -1;
    }

    return 0;
}

int SBGC::sendCommand(SBGCFrame &cmd)
{
    uint8_t start;
    int data_size_limit;

    // pre-check
    data_size_limit = SBGC_CMD_MAX_BYTES - SBGC_CMD_PAYLOAD_BYTES;
    if (cmd.data_size >= data_size_limit) {
        return -1;
    }

    // flush input
	this->serial.flushInput();

    // header
    start = 0x3E;  // ">" character
    this->serial.write(&start, 1);
    this->serial.write(&cmd.cmd_id, 1);
    this->serial.write(&cmd.data_size, 1);

    // body
    this->serial.write(&cmd.header_checksum, 1);
    this->serial.write(cmd.data, (size_t) cmd.data_size);
    this->serial.write(&cmd.data_checksum, 1);

    // flush output
	this->serial.flushOutput();

    return 0;
}

int SBGC::on(void)
{
	SBGCFrame cmd;
    cmd.buildCommand(CMD_MOTORS_ON);
	return this->sendCommand(cmd);
}

int SBGC::off(void)
{
    int retval;
	SBGCFrame cmd;
	uint8_t data[13];

    // turn off motor control
	data[0] = MODE_NO_CONTROL;
	for (int i = 1; i < 13; i++) {
        data[i] = 0;
    }
    cmd.buildCommand(CMD_CONTROL, data, 13);
	retval = this->sendCommand(cmd);
	if (retval != 0) {
        std::cout << "failed to turn motor control off!" << std::endl;
	}

    // turn off motors
    cmd.buildCommand(CMD_MOTORS_OFF);
	retval = this->sendCommand(cmd);
	if (retval != 0) {
        std::cout << "failed to turn motor control off!" << std::endl;
	}

    return 0;
}

int SBGC::reset(void)
{
    int retval;

    if (this->off() || this->on()) {
        std::cout << "failed to reset sbgc!" << std::endl;
        return -1;
    }

    return 0;
}

int SBGC::getBoardInfo(void)
{


    return 0;
}


int SBGC::setAngle(double roll, double pitch, double yaw)
{
	SBGCFrame cmd;
	int16_t roll_adjusted;
	int16_t pitch_adjusted;
	int16_t yaw_adjusted;
	uint8_t data[13];

	// adjust roll, pitch and yaw
	roll_adjusted = roll / FRAME_ANG_CONV;
	pitch_adjusted = pitch / FRAME_ANG_CONV;
	yaw_adjusted = yaw / FRAME_ANG_CONV;

    // control mode
	data[0] = MODE_ANGLE;

	// speed roll
	data[1] = 0;
	data[2] = 0;

	// angle roll
	data[3] = ((roll_adjusted >> 8) & 0xff);
	data[4] = ((roll_adjusted >> 0) & 0xff);

	// speed pitch
	data[5] = 0;
	data[6] = 0;

	// angle pitch
	data[7] = ((pitch_adjusted >> 8) & 0xff);
	data[8] = ((pitch_adjusted >> 0) & 0xff);

	// speed yaw
	data[9] = 0;
	data[10] = 0;

	// angle yaw
	data[11] = ((yaw_adjusted >> 8) & 0xff);
	data[12] = ((yaw_adjusted >> 0) & 0xff);

    // build frame and send
    cmd.buildCommand(CMD_CONTROL, data, 13);
	this->sendCommand(cmd);

    return 0;
}
