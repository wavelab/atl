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

void Quadrotor::runMission(
    Pose robot_pose,
    LandingTargetPosition landing_zone,
    float dt
)
{
    Position p;
    Eigen::Vector3d wp;
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;
    double elasped;

    // update pose
    this->updatePose(robot_pose);

    // mission
    switch (this->mission_state) {
    case IDLE_MODE:
        // transition to offboard mode
        // this->mission_state = INITIALIZE_MODE;
        this->mission_state = DISCOVER_MODE;
        this->hover_height = robot_pose.z + 3.0;

        // preset the landing zone position so when tag detection
        // is lost it will hover in the same spot
        this->going_to.x = this->pose.x;
        this->going_to.y = this->pose.y;
        this->going_to.z = this->hover_height;
        break;

    case DISCOVER_MODE:
        // tag detected?
        if (landing_zone.detected == true) {
            // check detection redunancy
            if (this->landing_zone_belief == 5) {
                std::cout << "Going to Carrot Tracker Mode!" << std::endl;
                this->mission_state = CARROT_TRACKER_MODE;
                this->landing_zone_belief = 0;

                // add start waypoint
                wp << this->pose.x, this->pose.y, this->pose.z;
                this->carrot_controller->wp_start = wp;
                this->carrot_controller->waypoints.push_back(wp);

                // add end waypoint
                wp << this->pose.x + landing_zone.x, this->pose.y + landing_zone.y, this->hover_height;
                this->carrot_controller->wp_end = wp;
                this->carrot_controller->waypoints.push_back(wp);

                // initialize carrot controller and wp_last_added
                this->carrot_controller->initialized = 1;
                this->wp_last_added = time(NULL);

            } else {
                this->landing_zone_belief++;

            }
        }

        p.x = this->going_to.x;
        p.y = this->going_to.y;
        p.z = this->hover_height;
        break;

    case CARROT_INITIALIZE_MODE:
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
            // this->mission_state = LAND_MODE;

        } else {
            p.x = carrot(0);
            p.y = carrot(1);
            p.z = carrot(2);

        }
        break;

    case TRACKING_MODE:
        // tag detected?
        if (landing_zone.detected == true) {
            // check detection redunancy
            if (this->landing_zone_belief < LZ_THRESHOLD) {
                this->landing_zone_belief++;

                p.x = this->going_to.x;
                p.y = this->going_to.y;
                p.z = this->hover_height;

            } else {
                p.x = this->pose.x + landing_zone.x;
                p.y = this->pose.y + landing_zone.y;
                p.z = this->hover_height;

                this->going_to.x = p.x;
                this->going_to.y = p.y;
                this->going_to.z = p.z;
            }

        } else {
            this->landing_zone_belief = 0;  // reset belief

            p.x = this->going_to.x;
            p.y = this->going_to.y;
            p.z = this->hover_height;

        }
        break;

    case CARROT_TRACKER_MODE:
        // add new waypoint
        elasped = difftime(time(NULL), this->wp_last_added);
        if (elasped > 1.0 && landing_zone.detected == true) {
            std::cout << "elapsed 1s!" << std::endl;
            // lower hover height if we are close to target
            // if ((this->pose.x - landing_zone.x) < 0.2 && (this->pose.x - landing_zone.x) < 0.2) {
            //     this->hover_height = this->hover_height * 0.9;
            // }

            wp <<
                this->pose.x + landing_zone.x,
                this->pose.y + landing_zone.y,
                this->hover_height;
            this->carrot_controller->waypoints.push_back(wp);
            this->wp_last_added = time(NULL);
        }

        // calculate new carrot
        position << this->pose.x, this->pose.y, this->pose.z;
        carrot << this->going_to.x,
                  this->going_to.y,
                  this->going_to.z;
        this->carrot_controller->update(position, carrot);

        // move to position
        p.x = carrot(0);
        p.y = carrot(1);
        p.z = carrot(2);

        // keep track of where it was going
        this->going_to.x = carrot(0),
        this->going_to.y = carrot(1),
        this->going_to.z = carrot(2);

        break;
    }

    // calcualte new attitude using position controller
    // std::cout << "move to: " << p.x << " " << p.y << std::endl;
    this->positionControllerCalculate(p, dt);
}
