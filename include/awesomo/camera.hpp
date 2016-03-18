#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#define _USE_MATH_DEFINES
#ifdef M_PI
  #define TWOPI 2*M_PI
#else
  #define TWOPI 2.0*3.1415926535897932384626433832795
#endif

#include <fstream>
#include <iostream>
#include <math.h>
#include <sys/time.h>

#include <ros/console.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <yaml-cpp/yaml.h>
#include <FlyCapture2.h>

// CONSTANTS
#define CAMERA_NORMAL 0
#define CAMERA_FIREFLY 1


class AprilTagPose
{
    public:
        int id;
        double distance;
        double yaw;
        double pitch;
        double roll;
        Eigen::Vector3d translation;
};

class Camera
{
    private:
        AprilTags::TagDetector *tag_detector;

        int camera_index;
        int camera_type;
        int image_width;
        int image_height;

        cv::VideoCapture *capture;
        FlyCapture2::Camera *capture_firefly;

        cv::Mat camera_matrix;
        cv::Mat rectification_matrix;
        cv::Mat distortion_coefficients;
        cv::Mat projection_matrix;

        void loadCalibrationFile(const std::string calibration_fp);
        int initializeNormalCamera();
        int initializeFireflyCamera();
        int getFrame(cv::Mat &image);
        void printFPS(double &last_tic, int &frame);
        float calculateFocalLength(void);
        double standardRad(double t);
        void convertToEuler(
            const Eigen::Matrix3d &wRo,
            double &yaw,
            double &pitch,
            double &roll
        );
        AprilTagPose obtainAprilTagPose(AprilTags::TagDetection& detection);
        void printDetection(AprilTags::TagDetection& detection);
        std::vector<AprilTagPose> processImage(cv::Mat &image, cv::Mat &image_gray);
        bool isFileEmpty(const std::string file_path);
        int outputAprilTagPose(const std::string output_fp, AprilTagPose &pose);

    public:
        vector<AprilTags::TagDetection> apriltags;
        std::vector<AprilTagPose> pose_estimates;

        Camera(int camera_index, int camera_type, const std::string calibration_fp);
        int run(void);
        std::vector<AprilTagPose> step(void);
        int runCalibration(void);
};

#endif
