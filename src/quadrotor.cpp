#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(std::map<std::string, std::string> configs)
{
    std::string config_path;

    // intialize state
    this->mission_state = IDLE_MODE;

    this->pose.x = 0.0;
    this->pose.y = 0.0;
    this->pose.z = 0.0;

    this->going_to.x = 0.0;
    this->going_to.y = 0.0;
    this->going_to.z = 0.0;

    this->hover_height = 0.0;
    this->landing_zone_belief = 0;

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

Attitude Quadrotor::positionControllerCalculate(Position setpoint, float dt)
{
    Attitude a;

    this->position_controller->calculate(setpoint, this->pose, dt);

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

void Quadrotor::initializeMission(void)
{
    Position p;
    Eigen::Vector3d wp;
    Eigen::Vector3d carrot;
    Eigen::Vector3d position;

    // current position + some altitude
    p.x = this->pose.x;
    p.y = this->pose.y;
    p.z = this->pose.z + 3;

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
}

void Quadrotor::runIdleMode(Pose robot_pose)
{
    std::cout << "IDEL MODE!" << std::endl;

    // transition to offboard mode
    this->mission_state = DISCOVER_MODE;
    this->hover_height = robot_pose.z + 3.0;

    // preset the landing zone position so when tag detection
    // is lost it will hover in the same spot
    this->going_to.x = this->pose.x;
    this->going_to.y = this->pose.y;
    this->going_to.z = this->hover_height;
}

Position Quadrotor::runHoverMode(Pose robot_pose)
{
    Position cmd;

    // pre-check
    if (this->hover_point_set == false) {
        this->hover_height = robot_pose.z;
        this->hover_point.x = robot_pose.x;
        this->hover_point.y = robot_pose.y;
        this->hover_point.z = this->hover_height;
        this->hover_point_set = true;
    }

    // command position
    cmd.x = this->hover_point.x;
    cmd.y = this->hover_point.y;
    cmd.z = this->hover_point.z;

    return cmd;
}

void Quadrotor::initializeCarrotController(void)
{
    std::cout << "Carrot controller initialized!" << std::endl;
    this->initializeMission();
    this->mission_state = CARROT_MODE;
}

Position Quadrotor::runCarrotMode(Pose robot_pose)
{
    Position cmd;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    position << robot_pose.x, robot_pose.y, robot_pose.z;
    if (this->carrot_controller->update(position, carrot) == 0) {
        std::cout << "No more waypoints!" << std::endl;
        // std::cout << "Landing!" << std::endl;
        // cmd.x = this->pose.x;
        // cmd.y = this->pose.y;
        // cmd.z = this->pose.z - 2;
        // // this->mission_state = LAND_MODE;

    } else {
        cmd.x = carrot(0);
        cmd.y = carrot(1);
        cmd.z = carrot(2);

    }

    return cmd;
}

Position Quadrotor::runKFDiscoverMode(Pose robot_pose, LandingTargetPosition landing_zone)
{
    Position cmd;
	Eigen::VectorXd mu(9);

    if (landing_zone.detected == true) {
        mu << this->pose.x + landing_zone.x,  // pos_x
              this->pose.y + landing_zone.y,  // pos_y
              this->pose.z + landing_zone.z,  // pos_z
              0.0, 0.0, 0.0,  // vel_x, vel_y, vel_z
              0.0, 0.0, 0.0;  // acc_x, acc_y, acc_z
        apriltag_kf_setup(&this->apriltag_estimator, mu);

        std::cout << "Going to KF Tracker Mode!" << std::endl;
        this->mission_state = TRACKING_MODE;

    } else {
        cmd.x = this->going_to.x;
        cmd.y = this->going_to.y;
        cmd.z = this->hover_height;

    }

    return cmd;
}

Position Quadrotor::runKFTrackingMode(Pose robot_pose, LandingTargetPosition landing_zone, float dt)
{
    Position cmd;
	Eigen::VectorXd y(3);
    double elasped;

    // transform detected tag to world frame
    y << robot_pose.x + landing_zone.x,
         robot_pose.y + landing_zone.y,
         robot_pose.z + landing_zone.z;

    // estimate tag position
    apriltag_kf_estimate(
        &this->apriltag_estimator,
        y,
        dt,
        landing_zone.detected
    );

    // build position
    cmd.x = this->apriltag_estimator.mu(0);
    cmd.y = this->apriltag_estimator.mu(1);
    cmd.z = this->hover_height;

    // keep track of target position
    if (landing_zone.detected == true) {
        this->going_to.x = cmd.x;
        this->going_to.y = cmd.y;
        this->going_to.z = cmd.z;

        // transition to landing
        elasped = difftime(time(NULL), this->tracking_start);
        if (elasped > 5 && landing_zone.x < 0.2 && landing_zone.y < 0.2) {
            this->mission_state = LANDING_MODE;
            this->height_last_updated = time(NULL);
            std::cout << "Landing!" << std::endl;

        }

    } else {
        cmd.x = this->going_to.x;
        cmd.y = this->going_to.y;
        cmd.z = this->going_to.z;

    }

    return cmd;
}

Position Quadrotor::runLandingMode(Pose robot_pose, LandingTargetPosition landing_zone, float dt)
{
    Position cmd;
	Eigen::VectorXd mu(9);
	Eigen::VectorXd y(3);
    double elasped;

    // transform detected tag to world frame
    y << this->pose.x + landing_zone.x,
         this->pose.y + landing_zone.y,
         this->pose.z + landing_zone.z;

    // estimate tag position
    apriltag_kf_estimate(
        &this->apriltag_estimator,
        y,
        dt,
        landing_zone.detected
    );

    // lower height
    elasped = difftime(time(NULL), this->height_last_updated);
    if (elasped > 1 && landing_zone.detected == true) {
        if (landing_zone.x < 0.3 && landing_zone.y < 0.3) {
            this->hover_height = this->hover_height * 0.7;
            printf("Lowering hover height to %f\n", this->hover_height);

        } else {
            this->hover_height += 0.5;
            printf("Increasing hover height to %f\n", this->hover_height);

        }

        this->height_last_updated = time(NULL);
    }

    // build position
    cmd.x = this->apriltag_estimator.mu(0);
    cmd.y = this->apriltag_estimator.mu(1);
    cmd.z = this->hover_height;

    // keep track of target position
    if (landing_zone.detected == true) {
        this->going_to.x = cmd.x;
        this->going_to.y = cmd.y;
        this->going_to.z = cmd.z;

        // kill engines (landed?)
        if (landing_zone.x < 0.2 && landing_zone.y < 0.2 && landing_zone.z < 0.4) {
            this->mission_state = LANDING_MODE;
        }

    } else {
        cmd.x = this->going_to.x;
        cmd.y = this->going_to.y;
        cmd.z = this->going_to.z;

    }

    return cmd;
}

int Quadrotor::runMission(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position position;

    // update pose
    this->updatePose(robot_pose);

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        this->runIdleMode(robot_pose);
        break;

    case HOVER_MODE:
        this->runHoverMode(robot_pose);
        break;

    case CARROT_INITIALIZE_MODE:
        this->initializeCarrotController();
        break;

    case CARROT_MODE:
        position = this->runCarrotMode(robot_pose);
        break;

    case DISCOVER_MODE:
        position = this->runKFDiscoverMode(robot_pose, landing_zone);
        break;

    case TRACKING_MODE:
        position = this->runKFTrackingMode(robot_pose, landing_zone, dt);
        break;

    case LANDING_MODE:
        position = this->runLandingMode(robot_pose, landing_zone, dt);
        break;

    case MISSION_ACCOMPLISHED:
        return 0;
        break;

    }

    // calcualte new attitude using position controller
    this->positionControllerCalculate(position, dt);

    return 1;
}
