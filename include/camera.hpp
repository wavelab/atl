#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

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

#include "util.hpp"
#include "apriltag.hpp"



// CONSTANTS
#define CAMERA_NORMAL 0
#define CAMERA_FIREFLY 1



// CLASSES
class CameraConfig
{
    public:
        int camera_mode;
        int image_width;
        int image_height;

        cv::Mat camera_matrix;
        cv::Mat rectification_matrix;
        cv::Mat distortion_coefficients;
        cv::Mat projection_matrix;
};

class Camera
{
    private:
        TagDetector *tag_detector;

        int camera_index;
        int camera_type;
        int camera_imshow;
        int camera_snapshot;
        std::string camera_mode;

        cv::Rect roi_rect;
        CameraConfig *config;
        std::map<std::string, CameraConfig *> configs;

        cv::VideoCapture *capture;
        FlyCapture2::Camera *capture_firefly;

        int initWebcam(int image_width, int image_height);
        int initFirefly();
        void printFPS(double &last_tic, int &frame);
        void adjustMode(std::vector<TagPose> &pose_estimates, int &timeout);

    public:
        vector<AprilTags::TagDetection> apriltags;
        std::vector<TagPose> pose_estimates;

        Camera(int camera_index, int camera_type);
        Camera(std::string camera_config_path);
        int initCamera(std::string camera_mode);
        CameraConfig *loadConfig(std::string mode, const std::string calib_file);
        int loadConfig(std::string camera_mode);
        int getFrame(cv::Mat &image);
        int run(void);
        std::vector<TagPose> step(int &timeout);
};

#endif
