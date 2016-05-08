#ifndef __APRILTAG_HPP__
#define __APRILTAG_HPP__

#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sys/time.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>

#include "awesomo/util.hpp"


#define _USE_MATH_DEFINES
#ifdef M_PI
  #define TWOPI 2*M_PI
#else
  #define TWOPI 2.0*3.1415926535897932384626433832795
#endif



// CLASSES
class TagPose
{
    public:
        int id;
        double distance;
        double yaw;
        double pitch;
        double roll;
        Eigen::Vector3d translation;
};

class TagDetector
{
    private:
        AprilTags::TagDetector *detector;
        cv::Rect roi_rect;
        int apriltag_imshow;

    public:
        TagDetector(void);
        TagDetector(int apriltag_imshow);
        void adjustROI(cv::Mat &image_gray, AprilTags::TagDetection &tag);
        std::vector<TagPose> processImage(
            cv::Mat &camera_matrix,
            cv::Mat &image,
            int &timeout
        );
        TagPose obtainPose(
            AprilTags::TagDetection &detection,
            cv::Mat camera_matrix
        );
        void printDetection(
            AprilTags::TagDetection &detection,
            cv::Mat camera_matrix
        );
};

#endif
