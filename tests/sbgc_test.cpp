#include "awesomo/munit.h"
#include "awesomo/sbgc.hpp"


// TEST FUNCTIONS
int testSBGCConnection(void);
int testSBGCGetBoardInfo(void);
int testSBGCSendCommand(void);


int testSBGCConnection(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);


    mu_check(sbgc.init() == 1);

    return 0;
}

int testSBGCGetBoardInfo(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

    mu_check(sbgc.init() == 1);
    sbgc.getBoardInfo();

    return 0;
}

int testSBGCSendCommand(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

    return 0;
}

int testSBGCSetAngle(void)
{
    SBGC sbgc("/dev/ttyUSB0", 115200, 500);

	sbgc.init();
	sbgc.setAngle(0, 10, 0);

	return 0;
}


void testSuite(void)
{
    // mu_add_test(testSBGCConnection);
    // mu_add_test(testSBGCGetBoardInformation);
    // mu_add_test(testSBGCSendCommand);
    mu_add_test(testSBGCSetAngle);
}

mu_run_tests(testSuite)
