#include "awesomo/quadrotor.hpp"


Quadrotor::Quadrotor(std::map<std::string, std::string> configs)
{
    std::string config_path;

    // intialize state
    this->mission_state = IDLE_MODE;

    this->pose.x = 0.0;
    this->pose.y = 0.0;
    this->pose.z = 0.0;

    this->landing_zone_prev.x = 0.0;
    this->landing_zone_prev.y = 0.0;
    this->landing_zone_prev.z = 0.0;

    this->landing_zone_world.x = 0.0;
    this->landing_zone_world.y = 0.0;
    this->landing_zone_world.z = 0.0;

    // initialize controllers
    if (configs.count("position_controller")) {
        config_path = configs["position_controller"];
        this->position_controller = new PositionController(config_path);
        std::cout << "position controller initialized!" << std::endl;
    } else {
        this->position_controller = NULL;
    }

    if (configs.count("carrot_controller")) {
        config_path = configs["carrot_controller"];
        this->carrot_controller = new CarrotController(config_path);
        std::cout << "carrot controller initialized!" << std::endl;
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

void Quadrotor::runMission(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position p;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;

    // update pose
    this->updatePose(robot_pose);

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        // transition to offboard mode
        // this->mission_state = INITIALIZE_MODE;
        this->mission_state = TRACKING_MODE;
        this->landing_zone_prev.x = landing_zone.x;
        this->landing_zone_prev.y = landing_zone.y;
        this->landing_zone_prev.z = landing_zone.z;
        this->landing_zone_world.x = this->pose.x;
        this->landing_zone_world.y = this->pose.y;
        this->landing_zone_world.z = 10;
        break;

    case INITIALIZE_MODE:
        this->initializeMission();
        this->mission_state = CARROT_MODE;
        std::cout << "Carrot controller initialized!" << std::endl;
        break;

    case CARROT_MODE:
        position << this->pose.x, this->pose.y, this->pose.z;
        if (this->carrot_controller->update(position, carrot) == 0) {
            std::cout << "Landing!" << std::endl;
            p.x = this->pose.x;
            p.y = this->pose.y;
            p.z = this->pose.z - 2;
            this->mission_state = LAND_MODE;

        } else {
            p.x = carrot(0);
            p.y = carrot(1);
            p.z = carrot(2);

        }
        break;

    case TRACKING_MODE:
        if (this->landing_zone_prev.x != landing_zone.x && this->landing_zone_prev.y != landing_zone.y) {
            p.x = this->pose.x + landing_zone.x;
            p.y = this->pose.y + landing_zone.y;
            p.z = 10;

            this->landing_zone_prev.x = landing_zone.x;
            this->landing_zone_prev.y = landing_zone.y;
            this->landing_zone_prev.z = landing_zone.z;

            this->landing_zone_world.x = p.x;
            this->landing_zone_world.y = p.y;
            this->landing_zone_world.z = 10;

        } else {
            p.x = this->landing_zone_world.x;
            p.y = this->landing_zone_world.y;
            p.z = 10;

        }
        break;

    case LAND_MODE:
        // do nothing
        break;
    }

    // calcualte new attitude using position controller
    this->positionControllerCalculate(p, dt);
}
