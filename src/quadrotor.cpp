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
    this->global_pose.x = 0.0;
    this->global_pose.y = 0.0;
    this->global_pose.z = 0.0;
    this->hover_point  = new HoverPoint();

    // landing state
    this->landing_zone_belief = 0;

    // estimators
    this->estimator_initialized = false;

    // initialize position controller
    if (configs.count("position_controller")) {
        config_path = configs["position_controller"];
        this->position_controller = new PositionController(config_path);
    } else {
        this->position_controller = NULL;
    }

    // initialize carrot controller
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
        this->hover_point->initialized = false; // later in the code this will be set
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
    this->global_pose.x = pose.x;
    this->global_pose.y = pose.y;
    this->global_pose.z = pose.z;

    this->global_pose.roll = pose.roll;
    this->global_pose.pitch = pose.pitch;
    this->global_pose.yaw = pose.yaw;
}

Attitude Quadrotor::positionControllerCalculate(
    Position setpoint,
    Pose robot_pose,
    float yaw,
    float dt,
    int frame
)
{
    Attitude a;

    this->position_controller->calculate(setpoint, robot_pose, yaw, dt, frame);

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
    std::cout << "IDLE MODE!" << std::endl;

    // transition to offboard mode
    this->mission_state = DISCOVER_MODE;

    // hover inplace
    if (this->hover_point->initialized == false) {
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x;
        this->hover_point->y = robot_pose.y;
        this->hover_point->z;  // configured in config file
    }
}

Position Quadrotor::runHoverMode(Pose robot_pose, float dt)
{
    Position cmd;
    float commanded_yaw = 0.0;
    // pre-check - if hover point not set, hover at current pose
    if (this->hover_point->initialized == false) {
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x;
        this->hover_point->y = robot_pose.y;
        this->hover_point->z;  // configured in config file
    }

    // command position
    cmd.x = this->hover_point->x;
    cmd.y = this->hover_point->y;
    cmd.z = this->hover_point->z;


    this->positionControllerCalculate(cmd, robot_pose, commanded_yaw,  dt, GLOBAL_FRAME);

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
    p.x = this->global_pose.x;
    p.y = this->global_pose.y;
    p.z = this->global_pose.z + 3;

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
    if (this->carrot_controller->update(position, carrot)) {
        std::cout << "No more waypoints!" << std::endl;
        std::cout << "Transitioning to Hover Mode!" << std::endl;

        this->mission_state = HOVER_MODE;

        this->hover_point->initialized = true;
        this->hover_point->x = this->carrot_controller->wp_end(0);
        this->hover_point->y = this->carrot_controller->wp_end(1);
        this->hover_point->z = this->carrot_controller->wp_end(2);


    } else {
        cmd = this->runHoverMode(robot_pose, dt);
        return cmd;
    }

    cmd.x = carrot(0);
    cmd.y = carrot(1);
    cmd.z = carrot(2);

    return cmd;
}

Position Quadrotor::runDiscoverMode(Pose robot_pose, LandingTargetPosition landing_zone)
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
    }

    return cmd;
}


Position Quadrotor::runTrackingModeBPF(
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position tag;
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
    tag.x = this->apriltag_estimator.mu(0);
    tag.y = this->apriltag_estimator.mu(1);
    tag.z = this->hover_point->z;

    // keep track of target position
    if (landing_zone.detected == true) {
        // update hover point
        this->hover_point->initialized = true;
        this->hover_point->x = landing_zone.x;
        this->hover_point->y = landing_zone.y;
        this->target_last_updated = time(NULL);
    }

    // printf(
    //     "estimation: %f %f %f\n",
    //     this->apriltag_estimator.mu(0),
    //     this->apriltag_estimator.mu(1),
    //     this->apriltag_estimator.mu(2)
    // );
    return tag;
}

Position Quadrotor::runLandingMode(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position tag;
    Position estimation;
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

    // build position command
    tag.x = this->apriltag_estimator.mu(0);
    tag.y = this->apriltag_estimator.mu(1);
    tag.z = this->hover_point->z;

    estimation.x = this->apriltag_estimator.mu(0);
    estimation.y = this->apriltag_estimator.mu(1);
    estimation.z = this->apriltag_estimator.mu(2);

    // printf(
    //     "estimation: %f %f %f\n",
    //     this->apriltag_estimator.mu(0),
    //     this->apriltag_estimator.mu(1),
    //     this->apriltag_estimator.mu(2)
    // );

    // keep track of target position
    if (landing_zone.detected == true) {
        this->hover_point->initialized = true;
        this->hover_point->x = robot_pose.x + tag.x;
        this->hover_point->y = robot_pose.y + tag.y;
        this->target_last_updated = time(NULL);

    }

    // lower height or increase height
    elasped = difftime(time(NULL), this->height_last_updated);
    if (elasped > 1 && landing_zone.detected == true) {
        if (landing_zone.x < 0.5 && landing_zone.y < 0.5) {
            this->hover_point->z = this->hover_point->z * 0.7;
            printf("Lowering hover height to %f\n", this->hover_point->z);

        } else {
            this->hover_point->z = this->hover_point->z * 1.2;
            printf("Increasing hover height to %f\n", this->hover_point->z);

        }

        this->height_last_updated = time(NULL);
    }

    // kill engines (landed?)
    if (landing_zone.x < 0.5 && landing_zone.y < 0.5 && landing_zone.z < 0.2) {
        printf("Mission Accomplished - disarming quadrotor!\n");
        this->mission_state = MISSION_ACCOMPLISHED;

    } else if (estimation.x < 0.5 && estimation.y < 0.5 && estimation.z < 0.2) {
        printf("Mission Accomplished - disarming quadrotor!\n");
        this->mission_state = MISSION_ACCOMPLISHED;

    }

    return tag;
}

int Quadrotor::followWaypoints(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position setpoint;
    Pose pose;
    float commanded_yaw = 0.0;
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
    this->positionControllerCalculate(setpoint, robot_pose, commanded_yaw,  dt, GLOBAL_FRAME);

    return 0;
}

int Quadrotor::runMission(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position tag_position;
    Position tag_origin;
    Pose fake_robot_pose;
    double target_lost_elasped;
    float roll;
    float pitch;
    float yaw;

    // calculate time elasped for target lost
    target_lost_elasped = difftime(time(NULL), this->target_last_updated);

    // update pose
    this->updatePose(robot_pose);

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        this->runIdleMode(robot_pose);
        this->runHoverMode(robot_pose, dt);
        return 1;
        break;

    case DISCOVER_MODE:
        this->runDiscoverMode(robot_pose, landing_zone);
        this->runHoverMode(robot_pose, dt);
        return 1;
        break;

    case TRACKING_MODE:
        tag_position = this->runTrackingModeBPF(landing_zone, dt);
        break;

    case LANDING_MODE:
        tag_position = this->runLandingMode(robot_pose, landing_zone, dt);
        break;

    case MISSION_ACCOMPLISHED:
        return 0;
        break;

    }

    // change from GPS to using AprilTag as world origin
    // modify setpoint and robot pose as we use GPS or AprilTag
    // swap robot pose with setpoint, since we are using AprilTag as
    // world origin, so now if
    //
    // this is now NED
    float commanded_yaw = 0.0;
    fake_robot_pose.x = 0.0;
    fake_robot_pose.y = 0.0;
    fake_robot_pose.z = 0.0 + robot_pose.z;  // don't desend this needs to be made a variable or something

    tag_origin.x = tag_position.x;
    tag_origin.y = tag_position.y;
    tag_origin.z = tag_position.z;

    if (landing_zone.detected) {
        // if the target has not been lost, set the global yaw?
        if (target_lost_elasped < TARGET_LOST_TIMEOUT) {
            commanded_yaw = this->global_pose.yaw;
        }
        this->positionControllerCalculate(
            tag_origin,
            fake_robot_pose,
            commanded_yaw,
            dt,
            BODY_PLANAR_FRAME
        );
    // target is not currently detected, but has not been lost for
    // more then two seconds
    } else if (target_lost_elasped < TARGET_LOST_TIMEOUT){
        this->positionControllerCalculate(
            tag_origin,
            fake_robot_pose,
            commanded_yaw,
            dt,
            BODY_PLANAR_FRAME
        );

    } else {
        this->runHoverMode(robot_pose, dt);
    }

    return this->mission_state;
}
