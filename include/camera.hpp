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
        int processImage(cv::Mat &image, cv::Mat &image_gray);

    public:
        int camera_index;
        int image_width;
        int image_height;
        vector<AprilTags::TagDetection> apriltags;

        Camera(void);
        int run(void);
};

#endif
