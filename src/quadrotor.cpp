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
    this->yaw = 0;

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

        // height offset
        this->height_offset_initialized = false;
        this->height_offset = 0.0f;

        // load hover height config
        this->hover_height_original = config["hover_height"].as<float>();
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
#ifdef YAW_CONTROL_ON
    this->position_controller->calculate(setpoint, robot_pose, yaw, dt);
#else
    this->position_controller->calculate(setpoint, robot_pose, 0.0, dt);
#endif

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

int Quadrotor::calculateLandingTargetYaw(double *yaw)
{
    int retval;
    double m;
    double c;
    double r;
    double x;
    double y;
    double relative_yaw;
    Eigen::Vector2d p;

    // perform linear regression to obtain line equation of
    // landing target of form: y = mx + c
    retval = linreg(this->lt_history, &m, &c, &r);
    if (retval == -1) {
        return -1;
    }

    // obtain relative yaw angle to quad
    p = this->lt_history[this->lt_history.size() - 1];
    y = p(1);
    x = (y - c) / m;
    relative_yaw = atan2(y, x);

    // convert relative yaw to global yaw
    *yaw = this->yaw + relative_yaw;
    if (*yaw < 0) {
        *yaw += 2 * M_PI;
    } else if (*yaw > 2 * M_PI) {
        *yaw -= 2 * M_PI;
    }

    return 0;
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
    p << 0.0, 0.0, this->hover_height;

    // waypoint 1
    wp = p;
    this->carrot_controller->wp_start = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 2
    wp(0) = p(0);
    wp(1) = p(1);
    wp(2) = p(2) * 0.8;
    this->carrot_controller->wp_end = wp;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 3
    wp(0) = p(0);
    wp(1) = p(1);
    wp(2) = p(2) * 0.6;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 4
    wp(0) = p(0);
    wp(1) = p(1);
    wp(2) = p(2) * 0.4;
    this->carrot_controller->waypoints.push_back(wp);

    // waypoint 5
    wp(0) = p(0);
    wp(1) = p(1);
    wp(2) = p(2) * 0.2;
    this->carrot_controller->waypoints.push_back(wp);

    // initialize carrot controller
    this->carrot_controller->initialized = 1;
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
        return this->carrot_controller->carrot_prev;
    }

    return carrot;
}

void Quadrotor::runDiscoverMode(LandingTargetPosition landing)
{
    Eigen::VectorXd mu(9);
    Eigen::Vector2d lt_pos;
    double lt_yaw;
    int retval;
    bool transition_state;

    // setup
    transition_state = false;

    // obtain landing target yaw
    if (landing.detected == true) {
        if (lt_history.size() < 5) {
            lt_pos << landing.position(0), landing.position(1);
            lt_history.push_back(lt_pos);

        } else if (lt_history.size() == 5) {
            // calculate landing target yaw
            retval = this->calculateLandingTargetYaw(&lt_yaw);
            if (retval == 0) {
                transition_state = true;
            }
            printf("Landing target yaw is: %.2f\n", lt_yaw);

            // set quadrotor yaw
            this->yaw = lt_yaw;

            // clear landing target history
            lt_history.clear();
        }
    }

    // initialize landing target estimator
    if (transition_state) {
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
        tic(&this->tracking_start);
        tic(&this->target_last_updated);

        // reset position controller softly so not initially violent
        this->position_controller->x.prev_error = landing.position(1);
        this->position_controller->y.prev_error = landing.position(0);
    }
}

void Quadrotor::runTrackingModeBPF(LandingTargetPosition landing, float dt)
{
    Eigen::Vector3d tag_mea;
    Eigen::Vector3d setpoint;
    Pose robot_pose;
    float tag_x;
    float tag_y;
    float elasped;

    // estimate tag position
    tag_mea = landing.position;
    apriltag_kf_estimate(&this->tag_estimator, tag_mea, dt, landing.detected);

    // keep track of target position
    elasped = mtoc(&this->target_last_updated);
    if (landing.detected == true) {
        tic(&this->target_last_updated);
    } else if (elasped > 1000) {
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
    this->positionControllerCalculate(setpoint, robot_pose, this->yaw, dt);

    // transition to landing
    elasped = mtoc(&this->tracking_start);
    if (elasped > (10 * 1000)) {  // track for 10 seconds then land
        printf("Transitioning to LANDING MODE!\n");
        this->mission_state = LANDING_MODE;
        tic(&this->height_last_updated);
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

int Quadrotor::checkLandingTargetEstimation(Eigen::Vector3d &est)
{
    if (est(0) > 5.0 || est(1) > 5.0) {
        return -1;
    }

    return 0;
}

void Quadrotor::runLandingMode(LandingTargetPosition landing, float dt)
{
    Eigen::Vector3d tag_mea;
    Eigen::Vector3d tag_est;
    Eigen::VectorXd mu(9);
    Eigen::Vector3d threshold;
    Pose robot_pose;
    float elasped;

    // estimate tag position
    tag_mea = landing.position;
    apriltag_kf_estimate(&this->tag_estimator, tag_mea, dt, landing.detected);
    mu = this->tag_estimator.mu;
    tag_est << mu(0), mu(1), this->hover_height;

    // check landing target estimation
    if (this->checkLandingTargetEstimation(tag_est) == -1) {
        printf("Target losted transitioning back to DISCOVER MODE!\n");
        this->mission_state = DISCOVER_MODE;

        // reset hover height and landing belief
        this->hover_height = this->hover_height_original;
        this->landing_belief = 0;
    }

    // keep track of target position
    elasped = mtoc(&this->target_last_updated);
    if (landing.detected == true) {
        tic(&this->target_last_updated);

    } else if (elasped > 1000.0) {
        printf("Target losted transitioning back to DISCOVER MODE!\n");
        this->mission_state = DISCOVER_MODE;

        // reset hover height and landing belief
        this->hover_height = this->hover_height_original;
        this->landing_belief = 0;

    }

    // landing - lower height or increase height
    elasped = mtoc(&this->height_last_updated);
    threshold = this->landing_config->cutoff_position;
    if (elasped > this->landing_config->period && landing.detected == true) {
        if (tag_mea(0) < threshold(0) && tag_mea(1) < threshold(1)) {
            this->hover_height *= this->landing_config->descend_multiplier;
            printf("Lowering hover height to %f\n", this->hover_height);

        } else {
            this->hover_height *= this->landing_config->recover_multiplier;
            printf("Increasing hover height to %f\n", this->hover_height);

        }
        tic(&this->height_last_updated);
    }

    // kill engines (landed?)
    if (this->withinLandingZone(tag_mea, tag_est)) {
        if (this->landing_belief >= this->landing_config->belief_threshold) {
            printf("MISSION ACCOMPLISHED!\n");
            this->mission_state = MISSION_ACCOMPLISHED;
        } else {
            this->landing_belief++;
        }
    }

    // robot pose
    robot_pose.position(0) = 0.0;
    robot_pose.position(1) = 0.0;
    robot_pose.position(2) = this->world_pose.position(2);
    robot_pose.q = this->world_pose.q;

    // update position controller
    this->positionControllerCalculate(tag_est, robot_pose, this->yaw, dt);
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
//         return 0;
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
        break;

    case DISCOVER_MODE:
        this->runDiscoverMode(landing);
        break;

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
