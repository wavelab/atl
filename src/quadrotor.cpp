#include "awesomo/quadrotor.hpp"


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
    this->mission_state = DISCOVER_MODE;
    this->world_pose.position << 0.0, 0.0, 0.0;

    // landing state
    this->landing_belief = 0;

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

        // load hover height config
        this->hover_height = config["hover_height"].as<float>();

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
        printf("ERROR! invalid quadrotor configuration file!\n");
        throw;
    }

    return 0;
}

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
    a.yaw = yaw;

    return a;
}

void Quadrotor::resetPositionController(void)
{
    this->position_controller->reset();
}

void Quadrotor::initializeCarrotController(void)
{
    Eigen::Vector3d p;
    Eigen::Vector3d wp;
    Eigen::Vector3d carrot;
    Eigen::Vector3d position;

    // setup
    std::cout << "Initializing Carrot Controller!" << std::endl;

    // current position
    p = this->world_pose.position;

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
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    // setup
    position = robot_pose.position;

    // calculate new carrot point
    if (this->carrot_controller->update(position, carrot)) {
        std::cout << "No more waypoints!" << std::endl;
        std::cout << "Transitioning to Hover Mode!" << std::endl;
        this->mission_state = HOVER_MODE;
    }

    return carrot;
}

void Quadrotor::runDiscoverMode(LandingTargetPosition landing)
{
	Eigen::VectorXd mu(9);

    if (landing.detected == true) {
        // initialize kalman filter
        mu << landing.position(0),  // pos_x relative to quad
              landing.position(1),  // pos_y relative to quad
              landing.position(2),  // pos_z relative to quad
              0.0, 0.0, 0.0,  // vel_x, vel_y, vel_z
              0.0, 0.0, 0.0;  // acc_x, acc_y, acc_z
        apriltag_kf_setup(&this->tag_estimator, mu);
        this->estimator_initialized = true;

        // transition to tracker mode
        printf("Transitioning to TRACKER MODE!\n");
        this->mission_state = TRACKING_MODE;
        this->tracking_start = time(NULL);
    }
}

void Quadrotor::runTrackingModeBPF(LandingTargetPosition landing, float dt)
{
    Eigen::Vector3d tag_mea;
    Eigen::Vector3d setpoint;
    Pose robot_pose;
    float tag_x;
    float tag_y;
    double elasped;

    // estimate tag position
    tag_mea = landing.position;
    apriltag_kf_estimate(&this->tag_estimator, tag_mea, dt, landing.detected);

    // keep track of target position
    elasped = difftime(time(NULL), this->target_last_updated);
    if (landing.detected == true) {
        this->target_last_updated = time(NULL);
    } else if (elasped > 2) {
        printf("Target losted transitioning back to DISCOVER MODE!\n");
        this->mission_state = DISCOVER_MODE;
    }

    // setpoint
    tag_x = this->tag_estimator.mu(0);
    tag_y = this->tag_estimator.mu(1);
    setpoint << tag_x, tag_y, this->hover_height;

    // robot pose
    robot_pose.position(0) = 0.0;
    robot_pose.position(1) = 0.0;
    robot_pose.position(2) = this->world_pose.position(2);
    robot_pose.q = this->world_pose.q;

    // update position controller
    this->positionControllerCalculate(setpoint, robot_pose, 0, dt);

    // transition to landing
    elasped = difftime(time(NULL), this->tracking_start);
    if (elasped > 10) {
        printf("Transitioning to LANDING MODE!\n");
        this->mission_state = LANDING_MODE;
    }
}

bool Quadrotor::withinLandingZone(Eigen::Vector3d &m, Eigen::Vector3d &e)
{
    Eigen::Vector3d threshold;
    bool measured_x_ok;
    bool measured_y_ok;
    bool measured_z_ok;
    bool estimated_x_ok;
    bool estimated_y_ok;
    bool estimated_z_ok;

    threshold = this->landing_config->cutoff_position;
    measured_x_ok = (m(0) < threshold(0)) ? true : false;
    measured_y_ok = (m(1) < threshold(1)) ? true : false;
    measured_z_ok = (m(2) < threshold(2)) ? true : false;
    estimated_x_ok = (e(0) < threshold(0)) ? true : false;
    estimated_y_ok = (e(1) < threshold(1)) ? true : false;
    estimated_z_ok = (e(2) < threshold(2)) ? true : false;

    // check measured
    if (measured_x_ok && measured_y_ok && measured_z_ok) {
        return true;

    // check estimated
    } else if (estimated_x_ok && estimated_y_ok && estimated_z_ok) {
        return true;

    // not near landing zone
    } else {
        return false;
    }
}

bool Quadrotor::withinLandingZone(Eigen::Vector3d &m)
{
    Eigen::Vector3d threshold;
    bool measured_x_ok;
    bool measured_y_ok;
    bool measured_z_ok;

    threshold = this->landing_config->cutoff_position;
    measured_x_ok = (m(0) < threshold(0)) ? true : false;
    measured_y_ok = (m(1) < threshold(1)) ? true : false;
    measured_z_ok = (m(2) < threshold(2)) ? true : false;

    // check measured
    if (measured_x_ok && measured_y_ok && measured_z_ok) {
        return true;

    // not near landing zone
    } else {
        return false;
    }
}

void Quadrotor::runLandingMode(LandingTargetPosition landing, float dt)
{
    Eigen::Vector3d tag_mea;
    Eigen::Vector3d tag_est;
	Eigen::VectorXd mu(9);
    Eigen::Vector3d threshold;
    double elasped;

    // estimate tag position
    tag_mea = landing.position;
    apriltag_kf_estimate(&this->tag_estimator, tag_mea, dt, landing.detected);
    mu = this->tag_estimator.mu;
    tag_est << mu(0), mu(1), this->hover_height;

    // keep track of target position
    if (landing.detected == true) {
        this->target_last_updated = time(NULL);
    }

    // landing - lower height or increase height
    elasped = difftime(time(NULL), this->height_last_updated);
    threshold = this->landing_config->cutoff_position;
    if (elasped > this->landing_config->period && landing.detected == true) {
        if (tag_mea(0) < threshold(0) && tag_mea(1) < threshold(1)) {
            this->hover_height *= this->landing_config->descend_multiplier;
            printf("Lowering hover height to %f\n", this->hover_height);

        } else {
            this->hover_height *= this->landing_config->recover_multiplier;
            printf("Increasing hover height to %f\n", this->hover_height);

        }
        this->height_last_updated = time(NULL);
    }

    // kill engines (landed?)
    if (this->withinLandingZone(tag_mea, tag_est)) {
        if (this->landing_belief > 5) {
            printf("MISSION ACCOMPLISHED!\n");
            this->mission_state = MISSION_ACCOMPLISHED;
        } else {
            this->landing_belief++;
        }
    }
}

// int Quadrotor::followWaypoints(
//     Pose robot_pose,
//     LandingTargetPosition landing,
//     float dt
// )
// {
//     Eigen::Vector3d setpoint;
//     Pose pose;
//     float commanded_yaw = 0.0;
//
//     // update pose
//     this->world_pose = robot_pose;
//
//     // mission
//     switch (this->mission_state) {
//     case CARROT_INITIALIZE_MODE:
//         this->initializeCarrotController();
//         setpoint = this->runHoverMode(robot_pose, dt);
//         break;
//
//     case CARROT_MODE:
//         setpoint = this->runCarrotMode(robot_pose, dt);
//         break;
//
//     case MISSION_ACCOMPLISHED:
//         return 1;
//         break;
//     }
//
//     // position controller calculate
//     this->positionControllerCalculate(setpoint, robot_pose, commanded_yaw,  dt);
//
//     return 0;
// }

int Quadrotor::runMission(
    Pose world_pose,
    LandingTargetPosition landing,
    float dt
)
{
    // update pose
    this->world_pose = world_pose;

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        this->resetPositionController();
        return this->mission_state;

    case DISCOVER_MODE:
        this->runDiscoverMode(landing);
        return this->mission_state;

    case TRACKING_MODE:
        this->runTrackingModeBPF(landing, dt);
        break;

    case LANDING_MODE:
        this->runLandingMode(landing, dt);
        break;

    case MISSION_ACCOMPLISHED:
        return 0;
        break;

    }

    return this->mission_state;
}
