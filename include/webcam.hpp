#ifndef __WEBCAM_H__
#define __WEBCAM_H__

#include <iostream>
#include <sys/time.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>


class Camera
{
    private:
        AprilTags::TagDetector* tag_detector;

        void printFPS(double &last_tic, int &frame);

    public:
        Camera();
        void run(void);
        int processImage(cv::Mat &image, cv::Mat &image_gray);
};

#endif
