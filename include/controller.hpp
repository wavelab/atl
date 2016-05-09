#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include <iostream>
#include <deque>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "util.hpp"


// CONSTANTS
// #define PID_CONFIG "/home/chutsu/Dropbox/proj/awesomo/configs/position_controller/pid.yaml"
#define PID_CONFIG "/home/odroid/catkin_ws/src/awesomo/configs/position_controller/pid.yaml"
// #define PID_CONFIG "/home/odroid/catkin_ws/src/awesomo/configs/position_controller/pid_semi_ok.yaml"



// STRUCTURES
struct pid
{
    int sample_rate;

    float setpoint;
    float output;

    float prev_error;
    float sum_error;

    float p_error;
    float i_error;
    float d_error;

    float k_p;
    float k_i;
    float k_d;

    float dead_zone;
    float min;
    float max;
};



// CLASSES
class CarrotController
{
    public:
        std::deque<Eigen::Vector3d> waypoints;
        int initialized;
        double look_ahead_dist;
        double wp_threshold;
        Eigen::Vector3d wp_start;
        Eigen::Vector3d wp_end;

        CarrotController();
        CarrotController(
            std::deque<Eigen::Vector3d> waypoints,
            double look_ahead_dist,
            double wp_threshold
        );
        CarrotController(std::string config_file_path);
        Eigen::Vector3d closestPoint(
            Eigen::Vector3d position,
            Eigen::Vector3d wp_start,
            Eigen::Vector3d wp_end
        );
        Eigen::Vector3d calculateCarrotPoint(
            Eigen::Vector3d position,
            double r,
            Eigen::Vector3d wp_start,
            Eigen::Vector3d wp_end
        );
        int waypointReached(
            Eigen::Vector3d position,
            Eigen::Vector3d waypoint,
            double threshold
        );
        int update(Eigen::Vector3d position, Eigen::Vector3d &carrot);
};

class PositionController
{
    public:
        struct pid x;
        struct pid y;
        struct pid T;

        float roll;
        float pitch;
        float throttle;
        float hover_throttle;

        // tf::Quaternion rpy_quat;
        float dt;

        PositionController(const std::string config_file);
        void loadConfig(const std::string config_file);
        void calculate(Pose p);
};

class AttitudeController
{
    public:
        struct pid motor_1;
        struct pid motor_2;
        struct pid motor_3;
        struct pid motor_4;

        float dt;

        AttitudeController(const std::string config_file);
        void loadConfig(const std::string config_file);
        void calculate(Pose p);
};

#endif
