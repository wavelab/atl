#include "awesomo/munit.h"
#include "awesomo/sbgc.hpp"


// TEST FUNCTIONS
int testSBGCConnection(void);
int testSBGCSendFrame(void);
int testSBGCReadFrame(void);
int testSBGCGetBoardInfo(void);


int testSBGCConnection(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);


    mu_check(sbgc.init() == 1);

    return 0;
}

int testSBGCSendFrame(void)
{
    int retval;
	SBGCFrame frame;
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

    // setup
    sbgc.init();

    // turn motors on
    frame.buildFrame(CMD_MOTORS_ON);
	retval = sbgc.sendFrame(frame);
	mu_check(retval == 0);
	sleep(1);

    // turn motors off
    frame.buildFrame(CMD_MOTORS_OFF);
	retval = sbgc.sendFrame(frame);
	mu_check(retval == 0);
	sleep(1);

    return 0;
}

int testSBGCReadFrame(void)
{
	int retval;
	SBGCFrame frame;
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

    // setup
    sbgc.init();

	// read frame
    frame.buildFrame(CMD_BOARD_INFO);
	sbgc.sendFrame(frame);
	retval = sbgc.readFrame(23, frame);

    // assert
	mu_check(frame.data_size == 18);
	mu_check(retval == 0);

	return 0;
}

int testSBGCGetBoardInfo(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

    mu_check(sbgc.init() == 1);
    sbgc.getBoardInfo();

    return 0;
}

int testSBGCSetAngle(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

	sbgc.init();
	sbgc.on();

	sbgc.setAngle(0, 10, 0);
	sleep(2);

	sbgc.off();

	return 0;
}


void testSuite(void)
{
    // mu_add_test(testSBGCConnection);
    // mu_add_test(testSBGCSendFrame);
    mu_add_test(testSBGCReadFrame);
    // mu_add_test(testSBGCGetBoardInformation);
    // mu_add_test(testSBGCSetAngle);
}

mu_run_tests(testSuite)
