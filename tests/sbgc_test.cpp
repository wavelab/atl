#include "awesomo/munit.h"
#include "awesomo/sbgc.hpp"


// TEST FUNCTIONS
int testSBGCConnectDisconnect(void);
int testSBGCSendFrame(void);
int testSBGCReadFrame(void);
int testSBGCGetBoardInfo(void);
int testSBGCGetRealtimeData(void);


int testSBGCConnectDisconnect(void)
{
    SBGC sbgc("/dev/ttyUSB0");
    mu_check(sbgc.connect() == 0);
    mu_check(sbgc.disconnect() == 0);

    return 0;
}

int testSBGCSendFrame(void)
{
    int retval;
    SBGCFrame frame;
    SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

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
    SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // read frame
    frame.buildFrame(CMD_BOARD_INFO);
    sbgc.sendFrame(frame);
    retval = sbgc.readFrame(CMD_BOARD_INFO_FRAME_SIZE, frame);

    // assert
    mu_check(frame.data_size == 18);
    mu_check(retval == 0);

    return 0;
}

int testSBGCGetBoardInfo(void)
{
    SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // test get board info
    sbgc.getBoardInfo();
    printf("board version: %d\n", sbgc.board_version);
    printf("firmware version: %d\n", sbgc.firmware_version);
    printf("debug mode: %d\n", sbgc.debug_mode);
    printf("board features: %d\n", sbgc.board_features);
    printf("connection flags: %d\n", sbgc.connection_flags);

    mu_check(sbgc.board_version == 30);
    mu_check(sbgc.firmware_version == 2569);

    return 0;
}

int testSBGCGetRealtimeData(void)
{
    SBGC sbgc("/dev/ttyUSB0");

    // setup
    sbgc.connect();

    // test get imu data
    for (int i = 0; i < 100; ++i){
        sbgc.getRealtimeData();
        sbgc.data.printData();
        // printf("roll %f \n", sbgc.data.rc_angles(0));
        // printf("pitch %f \n", sbgc.data.rc_angles(1));
        // printf("yaw %f \n", sbgc.data.rc_angles(2));
    }

    return 0;
}

int testSBGCSetAngle(void)
{
    SBGC sbgc("/dev/ttyUSB0");

    mu_check(sbgc.connect() == 0);
    sbgc.on();

    sbgc.setAngle(0, -90, 0);
    sleep(2);
    for (int angle = -95; angle < 20; angle += 3){
        sbgc.setAngle(0, angle, 0);
    }
    sbgc.off();

    return 0;
}

int testSBGCSetSpeedAngle(void)
{
    SBGC sbgc("/dev/ttyUSB0");

    mu_check(sbgc.connect() == 0);
    sbgc.on();

    sbgc.setSpeedAngle(0, 10, 0, 0, -2, 0);
    sleep(3);

    sbgc.off();

    return 0;
}


void testSuite(void)
{
    // mu_add_test(testSBGCConnectDisconnect);
    // mu_add_test(testSBGCSendFrame);
    // mu_add_test(testSBGCReadFrame);
    // mu_add_test(testSBGCGetBoardInfo);
    // mu_add_test(testSBGCGetRealtimeData);
    mu_add_test(testSBGCSetAngle);
    // mu_add_test(testSBGCSetSpeedAngle);
}

mu_run_tests(testSuite)
