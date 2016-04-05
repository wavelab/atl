#include "awesomo/camera.hpp"

#define FIREFLY_640 "/home/stan/Projects/awesomo/configs/pointgrey_firefly/ost_640.yaml"
#define FIREFLY_320 "/home/stan/Projects/awesomo/configs/pointgrey_firefly/ost_320.yaml"
#define FIREFLY_160 "/home/stan/Projects/awesomo/configs/pointgrey_firefly/ost_160.yaml"

int main(int argc, char **argv)
{

    // camera specifics
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig("default", FIREFLY_640);
    cam.loadConfig("320", FIREFLY_320);
    cam.loadConfig("160", FIREFLY_160);
    cam.initCamera("320");

    cam.photoMode();

    return 0;
}
