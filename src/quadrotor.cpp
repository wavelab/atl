#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(std::map<std::string, std::string> configs)
{
    std::string config_path;

    // precheck
    if (configs.count("quadrotor") == 0) {
        std::cout << "ERROR! quadrotor config not set!" << std::endl;
    }

    // configs
    this->tracking_config = new TrackingConfig();
    this->landing_config = new LandingConfig();

    // intialize state
    this->mission_state = IDLE_MODE;
    this->pose.x = 0.0;
    this->pose.y = 0.0;
    this->pose.z = 0.0;
    this->hover_point  = new HoverPoint();

    // landing state
    this->landing_zone_belief = 0;

    // estimators
    this->estimator_initialized = false;

    // position controller
    if (configs.count("position_controller")) {
        config_path = configs["position_controller"];
        this->position_controller = new PositionController(config_path);
    } else {
        this->position_controller = NULL;
    }

    // carrot controller
    if (configs.count("carrot_controller")) {
        config_path = configs["carrot_controller"];
        this->carrot_controller = new CarrotController(config_path);
    } else {
        this->carrot_controller = NULL;
    }

    // load quadrotor configuration
    config_path = configs["quadrotor"];
    this->loadConfig(config_path);
}

int Quadrotor::loadConfig(std::string config_file_path)
{
    YAML::Node config;
    YAML::Node tracking;
    YAML::Node landing;


    try {
        // load config
        config = YAML::LoadFile(config_file_path);

        // load hover point config
        this->hover_point->initialized = true;
        this->hover_point->x = 0.0f;
        this->hover_point->y = 0.0f;
        this->hover_point->z = config["hover_point"]["height"].as<float>();

        // load tracking config
        tracking = config["tracking"]["position_offset"];
        this->tracking_config->offset_x = tracking["x"].as<float>();
        this->tracking_config->offset_y = tracking["y"].as<float>();
        this->tracking_config->offset_z = tracking["z"].as<float>();

        // load landing config
        landing = config["landing"]["height_update"];
        this->landing_config->period = landing["period"].as<float>();

        landing = config["landing"]["height_update"]["multiplier"];
        this->landing_config->descend_multiplier = landing["descend"].as<float>();
        this->landing_config->recover_multiplier = landing["recover"].as<float>();

        landing = config["landing"]["disarm_conditions"];
        this->landing_config->x_cutoff = landing["x_cutoff"].as<float>();
        this->landing_config->y_cutoff = landing["y_cutoff"].as<float>();
        this->landing_config->z_cutoff = landing["z_cutoff"].as<float>();
        this->landing_config->belief_threshold = landing["belief_threshold"].as<float>();

    } catch (YAML::BadFile &ex) {
        throw;
    }

    return 0;
}

void Quadrotor::updatePose(Pose pose)
{
    this->pose.x = pose.x;
    this->pose.y = pose.y;
    this->pose.z = pose.z;

    this->pose.roll = pose.roll;
    this->pose.pitch = pose.pitch;
    this->pose.yaw = pose.yaw;
}

Attitude Quadrotor::positionControllerCalculate(
    Position setpoint,
    Pose robot_pose,
    float dt
)
{
    Attitude a;

    this->position_controller->calculate(setpoint, robot_pose, dt);

    a.x = this->position_controller->rpy_quat.x();
    a.y = this->position_controller->rpy_quat.y();
    a.z = this->position_controller->rpy_quat.z();
    a.w = this->position_controller->rpy_quat.w();

    a.roll = this->position_controller->roll;
    a.pitch = this->position_controller->pitch;
    a.yaw = 0;

    return a;
}

void Quadrotor::resetPositionController(void)
{
    this->position_controller->x.sum_error = 0.0;
    this->position_controller->x.prev_error = 0.0;
    this->position_controller->x.output = 0.0;

    this->position_controller->y.sum_error = 0.0;
    this->position_controller->y.prev_error = 0.0;
    this->position_controller->y.output = 0.0;

    this->position_controller->T.sum_error = 0.0;
    this->position_controller->T.prev_error = 0.0;
    this->position_controller->T.output = 0.0;
}

void Quadrotor::runIdleMode(Pose robot_pose)
{
    std::cout << "IDEL MODE!" << std::endl;

    // transition to offboard mode
    this->mission_state = DISCOVER_MODE;

    // hover inplace
    this->hover_point->initialized = true;
    this->hover_point->x = robot_pose.x;
    this->hover_point->y = robot_pose.y;
    this->hover_point->z = robot_pose.z + 3;
}

Position Quadrotor::runHoverMode(Pose robot_pose, float dt)
{
    Position cmd;

    // pre-check - if hover point not set, hover at current pose
    if (this->hover_point->initialized == false) {
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x;
        this->hover_point->y = robot_pose.y;
        this->hover_point->z = robot_pose.z;
    }

    // command position
    cmd.x = this->hover_point->x;
    cmd.y = this->hover_point->y;
    cmd.z = this->hover_point->z;

    // position controller calculate
    this->positionControllerCalculate(cmd, robot_pose, dt);

    return cmd;
}

void Quadrotor::initializeCarrotController(void)
{
    Position p;
    Eigen::Vector3d wp;
    Eigen::Vector3d carrot;
    Eigen::Vector3d position;

    // setup
    std::cout << "Initializing Carrot Controller!" << std::endl;

    // current position + some altitude
    p.x = this->pose.x;
    p.y = this->pose.y;
    p.z = this->hover_point->z;

    // waypoint 1
    wp << p.x, p.y, p.z;
    this->carrot_controller->wp_start = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 2
    wp << p.x + 5, p.y, p.z;
    this->carrot_controller->wp_end = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 3
    wp << p.x + 5, p.y + 5, p.z;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 4
    wp << p.x, p.y + 5, p.z;
    this->carrot_controller->waypoints.push_back(wp);

    // back to waypoint 1
    wp << p.x, p.y, p.z;
    this->carrot_controller->waypoints.push_back(wp);

    // initialize carrot controller
    this->carrot_controller->initialized = 1;
    this->mission_state = CARROT_MODE;
    std::cout << "Transitioning to Carrot Controller Mode!" << std::endl;
}

Position Quadrotor::runCarrotMode(Pose robot_pose, float dt)
{
    Position cmd;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    // setup
    position << robot_pose.x, robot_pose.y, robot_pose.z;

    // calculate new carrot point
    if (this->carrot_controller->update(position, carrot) == 0) {
        std::cout << "No more waypoints!" << std::endl;
        std::cout << "Transitioning to Hover Mode!" << std::endl;
        this->hover_point->initialized = true;
        this->hover_point->x = this->carrot_controller->wp_end(0);
        this->hover_point->y = this->carrot_controller->wp_end(1);
        this->hover_point->z = this->carrot_controller->wp_end(2);
        this->mission_state = HOVER_MODE;

    } else {
        cmd = this->runHoverMode(robot_pose, dt);

    }

    return cmd;
}

Position Quadrotor::runDiscoverMode(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position cmd;
	Eigen::VectorXd mu(9);

    if (landing_zone.detected == true) {
        // initialize kalman filter
        mu << landing_zone.x,  // pos_x relative to quad
              landing_zone.y,  // pos_y relative to quad
              landing_zone.z,  // pos_z relative to quad
              0.0, 0.0, 0.0,  // vel_x, vel_y, vel_z
              0.0, 0.0, 0.0;  // acc_x, acc_y, acc_z
        apriltag_kf_setup(&this->apriltag_estimator, mu);
        this->estimator_initialized = true;

        // transition to tracker mode
        std::cout << "Transitioning to Tracker Mode!" << std::endl;
        this->mission_state = TRACKING_MODE;
        this->tracking_start = time(NULL);

        // keep track of hover point
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x + landing_zone.x;
        this->hover_point->y = robot_pose.y + landing_zone.y;

        cmd.x = this->hover_point->x;
        cmd.y = this->hover_point->y;
        cmd.z = this->hover_point->z;
    }

    // hover in-place
    cmd = this->runHoverMode(robot_pose, dt);

    return cmd;
}

Position Quadrotor::runTrackingMode(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position cmd;
    Position tag_position;
	Eigen::VectorXd y(3);
    double elasped;

    // measured landing zone x, y, z
    y << landing_zone.x, landing_zone.y, landing_zone.z;

    // estimate tag position
    apriltag_kf_estimate(
        &this->apriltag_estimator,
        y,
        dt,
        landing_zone.detected
    );

    // build position command
    tag_position.x = this->apriltag_estimator.mu(0);
    tag_position.y = this->apriltag_estimator.mu(1);
    tag_position.z = this->hover_point->z;  // don't desend, yet

    // keep track of target position
    if (landing_zone.detected == true) {
        // update hover point
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x + tag_position.x;
        this->hover_point->y = robot_pose.y + tag_position.y;

        cmd.x = this->hover_point->x;
        cmd.y = this->hover_point->y;
        cmd.z = this->hover_point->z;

        // transition to landing?
        elasped = difftime(time(NULL), this->tracking_start);
        if (elasped > 3 && landing_zone.x < 2.0 && landing_zone.y < 2.0) {
            this->mission_state = LANDING_MODE;
            this->height_last_updated = time(NULL);
            std::cout << "Transitioning to Landing Mode!" << std::endl;

        }

        // modify setpoint and robot pose as we use GPS or AprilTag
        // swap robot pose with setpoint, since we are using AprilTag as
        // world origin, so now if
        robot_pose.x = -tag_position.x;
        robot_pose.y = tag_position.y;
        robot_pose.z = tag_position.z;

        tag_position.x = 0.0f;
        tag_position.y = 0.0f;
        tag_position.z = robot_pose.z;

        // calcualte new attitude using position controller
        this->positionControllerCalculate(tag_position, robot_pose, dt);

    } else {
        cmd = this->runHoverMode(robot_pose, dt);

    }

    return cmd;
}

Position Quadrotor::runLandingMode(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position cmd;
    Position tag_position;
	Eigen::VectorXd mu(9);
	Eigen::VectorXd y(3);
    double elasped;

    // measured landing zone x, y, z
    y << landing_zone.x, landing_zone.y, landing_zone.z;

    // estimate tag position
    apriltag_kf_estimate(
        &this->apriltag_estimator,
        y,
        dt,
        landing_zone.detected
    );

    // lower height
    elasped = difftime(time(NULL), this->height_last_updated);
    if (elasped > 2 && landing_zone.detected == true) {
        if (landing_zone.x < 2 && landing_zone.y < 2) {
            this->hover_point->z = this->hover_point->z * 0.8;
            printf("Lowering hover height to %f\n", this->hover_point->z);

        } else {
            this->hover_point->z += 0.5;
            printf("Increasing hover height to %f\n", this->hover_point->z);

        }

        this->height_last_updated = time(NULL);
    }

    // build position command
    tag_position.x = this->apriltag_estimator.mu(0);
    tag_position.y = this->apriltag_estimator.mu(1);
    tag_position.z = this->hover_point->z;

    // keep track of target position
    if (landing_zone.detected == true) {
        // update hover point
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x + tag_position.x;
        this->hover_point->y = robot_pose.y + tag_position.y;

        cmd.x = this->hover_point->x;
        cmd.y = this->hover_point->y;
        cmd.z = this->hover_point->z;

        // kill engines (landed?)
        if (landing_zone.x < 0.5 && landing_zone.y < 0.5 && landing_zone.z < 0.4) {
            this->landing_zone_belief += 1;

            if (this->landing_zone_belief == 10) {
                printf("Mission Accomplished!\n");
                this->mission_state = MISSION_ACCOMPLISHED;
            }
        }

        // modify setpoint and robot pose as we use GPS or AprilTag
        // swap robot pose with setpoint, since we are using AprilTag as
        // world origin, so now if
        robot_pose.x = -tag_position.x;
        robot_pose.y = tag_position.y;
        robot_pose.z = tag_position.z;  // don't desend

        tag_position.x = 0.0f;
        tag_position.y = 0.0f;
        tag_position.z = robot_pose.z;

        // calcualte new attitude using position controller
        this->positionControllerCalculate(tag_position, robot_pose, dt);

    } else {
        cmd = this->runHoverMode(robot_pose, dt);

    }

    return cmd;
}

int Quadrotor::followWaypoints(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position setpoint;
    Pose pose;

    // update pose
    this->updatePose(robot_pose);

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        this->runIdleMode(robot_pose);
        setpoint = this->runHoverMode(robot_pose, dt);
        break;

    case HOVER_MODE:
        setpoint = this->runHoverMode(robot_pose, dt);
        break;

    case CARROT_INITIALIZE_MODE:
        this->initializeCarrotController();
        setpoint = this->runHoverMode(robot_pose, dt);
        break;

    case CARROT_MODE:
        setpoint = this->runCarrotMode(robot_pose, dt);
        break;

    case MISSION_ACCOMPLISHED:
        return 1;
        break;

    }

    // position controller calculate
    this->positionControllerCalculate(setpoint, robot_pose, dt);

    return 0;
}

int Quadrotor::runMission(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position tag_position;
    Pose pose;

    // update pose
    this->updatePose(robot_pose);

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        this->runIdleMode(robot_pose);
        this->runHoverMode(robot_pose, dt);
        break;

    case HOVER_MODE:
        this->runHoverMode(robot_pose, dt);
        break;

    case DISCOVER_MODE:
        this->runDiscoverMode(robot_pose, landing_zone, dt);
        break;

    case TRACKING_MODE:
        this->runTrackingMode(robot_pose, landing_zone, dt);
        break;

    case LANDING_MODE:
        this->runLandingMode(robot_pose, landing_zone, dt);
        break;

    case MISSION_ACCOMPLISHED:
        return 0;
        break;

    }

    return 1;
}
