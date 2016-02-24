#ifndef __WEBCAM_H__
#define __WEBCAM_H__

#define _USE_MATH_DEFINES
#ifdef M_PI
  #define TWOPI 2*M_PI
#else
  #define TWOPI 2.0*3.1415926535897932384626433832795
#endif

#include <iostream>
#include <math.h>
#include <sys/time.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>

#include <yaml-cpp/yaml.h>


class Camera
{
    private:
        AprilTags::TagDetector* tag_detector;

        int camera_index;
        int image_width;
        int image_height;

        cv::Mat camera_matrix;
        cv::Mat rectification_matrix;
        cv::Mat distortion_coefficients;
        cv::Mat projection_matrix;

        void loadCalibrationFile(const std::string calibration_fp);
        void printFPS(double &last_tic, int &frame);
        float calculateFocalLength(void);
        double standardRad(double t);
        void convertToEuler(
            const Eigen::Matrix3d &wRo,
            double &yaw,
            double &pitch,
            double &roll
        );
        void printDetection(AprilTags::TagDetection& detection);
        int processImage(cv::Mat &image, cv::Mat &image_gray);

    public:
        vector<AprilTags::TagDetection> apriltags;

        Camera(int camera_index, const std::string calibration_fp);
        int run(void);
};

#endif
