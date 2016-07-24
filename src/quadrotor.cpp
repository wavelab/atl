#include "awesomo/quadrotor.hpp"

HoverPoint::HoverPoint()
{
    this->initialized = false;
    this->position << 0.0, 0.0, 0.0;
}


HoverPoint::HoverPoint(float x, float y, float z)
{
    this->initialized = true;
    this->position << x, y, z;
}


LandingConfig::LandingConfig(void)
{
        this->period = 0;
        this->descend_multiplier = 0;
        this->recover_multiplier = 0;
        this->cutoff_position <<  0, 0, 0;
        this->belief_threshold = 0;
}


LandingConfig::LandingConfig(
    float period,
    float desend_multiplier,
    float recover_multiplier,
    float belief_threshold,
    Eigen::Vector3d cutoff_position
)
{
        this->period = period;
        this->descend_multiplier = descend_multiplier;
        this->recover_multiplier = recover_multiplier;
        this->cutoff_position = cutoff_position;
        this->belief_threshold = belief_threshold;
}


Quadrotor::Quadrotor(std::map<std::string, std::string> configs)
{
    std::string config_path;

    // precheck
    if (configs.count("quadrotor") == 0) {
        std::cout << "ERROR! quadrotor config not set!" << std::endl;
    }

    // configs
    this->landing_config = new LandingConfig();

    // intialize state
    this->mission_state = IDLE_MODE;
    this->global_pose.position << 0.0, 0.0, 0.0;
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
        this->hover_point->position <<
            0.0f, 0.0f, 0.0f;
        this->hover_point->hover_height = config["hover_point"]["height"].as<float>();

        // load landing config
        landing = config["landing"]["height_update"];
        this->landing_config->period = landing["period"].as<float>();

        landing = config["landing"]["height_update"]["multiplier"];
        this->landing_config->descend_multiplier = landing["descend"].as<float>();
        this->landing_config->recover_multiplier = landing["recover"].as<float>();

        landing = config["landing"]["disarm_conditions"];
        this->landing_config->cutoff_position <<
            landing["x_cutoff"].as<float>(),
            landing["y_cutoff"].as<float>(),
            landing["z_cutoff"].as<float>();

        this->landing_config->belief_threshold =
            landing["belief_threshold"].as<float>();

    } catch (YAML::BadFile &ex) {
        throw;
    }

    return 0;
}

// void Quadrotor::updatePose(Pose pose)
// {
//     this->global_pose.x = pose.x;
//     this->global_pose.y = pose.y;
//     this->global_pose.z = pose.z;
//
//     this->global_pose.roll = pose.roll;
//     this->global_pose.pitch = pose.pitch;
//     this->global_pose.yaw = pose.yaw;
// }

Attitude Quadrotor::positionControllerCalculate(
    Eigen::Vector3d setpoint,
    Pose robot_pose,
    float yaw,
    float dt
)
{
    Attitude a;
    this->position_controller->calculate(setpoint, robot_pose, yaw, dt);

    a.x = this->position_controller->command_quat.x();
    a.y = this->position_controller->command_quat.y();
    a.z = this->position_controller->command_quat.z();
    a.w = this->position_controller->command_quat.w();

    a.roll = this->position_controller->roll;
    a.pitch = this->position_controller->pitch;
    a.yaw = 0;

    return a;
}

void Quadrotor::resetPositionController(void)
{
    this->position_controller->reset();
}

void Quadrotor::runIdleMode(Pose robot_pose)
{
    std::cout << "IDLE MODE!" << std::endl;

    // transition to offboard mode
    this->mission_state = DISCOVER_MODE;

    // hover inplace
    if (this->hover_point->initialized == false) {
        this->hover_point->initialized = true;
        this->hover_point->position = robot_pose.position +
            Eigen::Vector3d(0, 0, this->hover_point->hover_height);
    }
}

Eigen::Vector3d Quadrotor::runHoverMode(Pose robot_pose, float dt)
{
    Eigen::Vector3d cmd_position;
    float commanded_yaw = 0.0;
    // pre-check - if hover point not set, hover at current pose
    if (this->hover_point->initialized == false) {
        this->hover_point->initialized = true;
        this->hover_point->position = robot_pose.position +
            Eigen::Vector3d(0, 0, this->hover_point->hover_height);
    }

    // command position
    cmd_position = this->hover_point->position;

    // set up to send a mavros set position
    this->positionControllerCalculate(cmd_position, robot_pose, commanded_yaw,  dt);

    return cmd_position;
}

void Quadrotor::initializeCarrotController(void)
{
    Eigen::Vector3d p;
    Eigen::Vector3d wp;
    Eigen::Vector3d carrot;
    Eigen::Vector3d position;

    // setup
    std::cout << "Initializing Carrot Controller!" << std::endl;

    // current position + some altitude
    p = this->global_pose.position;
    // p.x = this->global_pose.x;
    // p.y = this->global_pose.y;
    // p.z = this->global_pose.z + 3;

    // waypoint 1
    wp = p;
    this->carrot_controller->wp_start = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 2
    wp(0) = p(0) + 5;
    wp(1) = p(1);
    wp(2) = p(2);

    this->carrot_controller->wp_end = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 3
    wp(0) = p(0) + 5;
    wp(1) = p(1) + 5;
    wp(2) = p(2);
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 4
    wp(0) = p(0);
    wp(1) = p(1) + 5;
    wp(2) = p(2);
    this->carrot_controller->waypoints.push_back(wp);

    // back to waypoint 1
    wp = p;
    this->carrot_controller->waypoints.push_back(wp);

    // initialize carrot controller
    this->carrot_controller->initialized = 1;
    this->mission_state = CARROT_MODE;
    std::cout << "Transitioning to Carrot Controller Mode!" << std::endl;
}

Eigen::Vector3d Quadrotor::runCarrotMode(Pose robot_pose, float dt)
{
    Eigen::Vector3d cmd_position;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    // setup
    position = robot_pose.position;

    // calculate new carrot point
    if (this->carrot_controller->update(position, carrot)) {
        std::cout << "No more waypoints!" << std::endl;
        std::cout << "Transitioning to Hover Mode!" << std::endl;

        this->mission_state = HOVER_MODE;

        this->hover_point->initialized = true;
        this->hover_point->position = this->carrot_controller->wp_end;


    } else {
        cmd_position = this->runHoverMode(robot_pose, dt);
        return cmd_position;
    }

    cmd_position = carrot;
    return cmd_position;
}

Eigen::Vector3d Quadrotor::runDiscoverMode(Pose robot_pose, LandingTargetPosition landing_zone)
{
    Eigen::Vector3d cmd_position;
	Eigen::VectorXd mu(9);

    if (landing_zone.detected == true) {
        // initialize kalman filter
        mu << landing_zone.position(0),  // pos_x relative to quad
              landing_zone.position(1),  // pos_y relative to quad
              landing_zone.position(2),  // pos_z relative to quad
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
        this->hover_point->position(0) = robot_pose.position(0) + landing_zone.position(0);
        this->hover_point->position(1) = robot_pose.position(1) + landing_zone.position(1);
    }

    return cmd_position;
}


Eigen::Vector3d Quadrotor::runTrackingModeBPF(
    LandingTargetPosition landing_zone,
    float dt
)
{
    Eigen::Vector3d tag_position;
	Eigen::VectorXd y(3);
    double elasped;

    // measured landing zone x, y, z
    y = landing_zone.position ;

    // estimate tag position
    apriltag_kf_estimate(
        &this->apriltag_estimator,
        y,
        dt,
        landing_zone.detected
    );

    // build position command
    tag_position = Eigen::Vector3d(this->apriltag_estimator.mu(0),
        this->apriltag_estimator.mu(1),
        this->hover_point->position(2)
    );

    // keep track of target position
    if (landing_zone.detected == true) {
        // update hover point
        this->hover_point->initialized = true;
        this->hover_point->position = Eigen::Vector3d(
            y(0), y(1), this->hover_point->position(2)
        );
        this->target_last_updated = time(NULL);
    }

    // printf(
    //     "estimation: %f %f %f\n",
    //     this->apriltag_estimator.mu(0),
    //     this->apriltag_estimator.mu(1),
    //     this->apriltag_estimator.mu(2)
    // );
    return tag_position;
}

Eigen::Vector3d Quadrotor::runLandingMode(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Eigen::Vector3d tag_position;
    Eigen::Vector3d tag_estimation;
	Eigen::VectorXd mu(9);
	Eigen::VectorXd y(3);
    double elasped;

    // measured landing zone x, y, z
    y = landing_zone.position;

    // estimate tag position
    apriltag_kf_estimate(
        &this->apriltag_estimator,
        y,
        dt,
        landing_zone.detected
    );

    // build position command
    tag_position << this->apriltag_estimator.mu(0),
        this->apriltag_estimator.mu(1),
        this->hover_point->position(2);

    tag_estimation = this->apriltag_estimator.mu;

    // printf(
    //     "estimation: %f %f %f\n",
    //     this->apriltag_estimator.mu(0),
    //     this->apriltag_estimator.mu(1),
    //     this->apriltag_estimator.mu(2)
    // );

    // keep track of target position
    if (landing_zone.detected == true) {
        // update hover point
        this->hover_point->initialized = true;
        this->hover_point->position << y(0), y(1), this->hover_point->position(2);
        this->target_last_updated = time(NULL);
    }

    // lower height or increase height
    elasped = difftime(time(NULL), this->height_last_updated);
    if (elasped > 1 && landing_zone.detected == true) {
        if (landing_zone.position(0) < 0.5 && landing_zone.position(1) < 0.5) {
            this->hover_point->position(2) *=  0.7;
            printf("Lowering hover height to %f\n", this->hover_point->position(2));

        } else {
            this->hover_point->position(2) *=  1.2;
            printf("Increasing hover height to %f\n", this->hover_point->position(2));

        }
        this->height_last_updated = time(NULL);
    }

    // kill engines (landed?)
    if (landing_zone.position(0) < 0.5 &&
        landing_zone.position(1) < 0.5 &&
        landing_zone.position(2) < 0.2
    )
    {
        printf("Mission Accomplished - disarming quadrotor!\n");
        this->mission_state = MISSION_ACCOMPLISHED;

    } else if (tag_estimation(0) < 0.5 &&
            tag_estimation(1) < 0.5 &&
            tag_estimation(2) < 0.2) {
        printf("Mission Accomplished - disarming quadrotor!\n");
        this->mission_state = MISSION_ACCOMPLISHED;

    }

    return tag_estimation;
}

int Quadrotor::followWaypoints(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Eigen::Vector3d setpoint;
    Pose pose;
    float commanded_yaw = 0.0;
    // update pose
    this->global_pose = robot_pose;

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
    this->positionControllerCalculate(setpoint, robot_pose, commanded_yaw,  dt);

    return 0;
}

int Quadrotor::runMission(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Eigen::Vector3d tag_position;
    Eigen::Vector3d tag_origin;
    Pose fake_robot_pose;
    double target_lost_elasped;
    float roll;
    float pitch;
    float yaw;

    // calculate time elasped for target lost
    target_lost_elasped = difftime(time(NULL), this->target_last_updated);

    // update pose
    this->global_pose = robot_pose;

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
    float commanded_yaw = 0.0;
    fake_robot_pose.position << 0.0, 0, 0;
    fake_robot_pose.position(2) = 0.0 + robot_pose.position(2);
    // don't desend this needs to be made a variable or something

    // there landing mode is not coded at the momeent? ??
    tag_origin = tag_position;

    if (landing_zone.detected) {
        // if the target has not been lost, set the global yaw?
        if (target_lost_elasped < TARGET_LOST_TIMEOUT) {
            commanded_yaw = 0.0;
        }
        this->positionControllerCalculate(
            tag_origin,
            fake_robot_pose,
            commanded_yaw,
            dt
        );
    // target is not currently detected, but has not been lost for
    // more then two seconds
    } else if (target_lost_elasped < TARGET_LOST_TIMEOUT){
        this->positionControllerCalculate(
            tag_origin,
            fake_robot_pose,
            commanded_yaw,
            dt
        );

    } else {
        this->runHoverMode(robot_pose, dt);
    }

    return this->mission_state;
}
