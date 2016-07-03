#include "awesomo/controller.hpp"


// CARROT CONTROLLER
CarrotController::CarrotController() : initialized(0) { }

CarrotController::CarrotController(
    std::deque<Eigen::Vector3d> waypoints,
    double look_ahead_dist,
    double wp_threshold
)
{
    if (waypoints.size() > 0) {
        this->initialized = 1;
    }
    this->waypoints = waypoints;
    this->look_ahead_dist = look_ahead_dist;
    this->wp_threshold = wp_threshold;
}

CarrotController::CarrotController(std::string config_file_path)
{
    YAML::Node carrot_config;
    Eigen::Vector3d position;

    carrot_config = YAML::LoadFile(config_file_path);
    this->look_ahead_dist = carrot_config["look_ahead_dist"].as<float>();
    this->wp_threshold = carrot_config["wp_threshold"].as<float>();

    for (int i = 0; i < carrot_config["waypoints"].size(); i += 3) {
        position <<
            carrot_config["waypoints"][i].as<float>(),
            carrot_config["waypoints"][i + 1].as<float>(),
            carrot_config["waypoints"][i + 2].as<float>();

        this->waypoints.push_back(position);
    }
    if (waypoints.size() > 0) {
        this->initialized = 1;
    }
}

Eigen::Vector3d CarrotController::closestPoint(
    Eigen::Vector3d position,
    Eigen::Vector3d wp_start,
    Eigen::Vector3d wp_end
)
{
    double t;
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;

	// calculate closest point
	v1 = position - wp_start;
	v2 = wp_end - wp_start;
	t = v1.dot(v2) / v2.squaredNorm();

    // make sure the point is between wp_start and wp_end
	if (t < 0) {
		return wp_start;
    } else if (t > 1) {
		return wp_end;
    }

    // result
    return wp_start + t * v2;
}

Eigen::Vector3d CarrotController::calculateCarrotPoint(
    Eigen::Vector3d position,
    double r,
    Eigen::Vector3d wp_start,
    Eigen::Vector3d wp_end
)
{
    Eigen::Vector3d u;
    Eigen::Vector3d v;
    Eigen::Vector3d pt_on_line;

    // get closest point
    pt_on_line = this->closestPoint(position, wp_start, wp_end);

    // calculate carrot point on wp_start and wp_end
    v = wp_end - wp_start;
    u = v / v.norm();
    return pt_on_line + r * u;
}

int CarrotController::waypointReached(
    Eigen::Vector3d position,
    Eigen::Vector3d waypoint,
    double threshold
)
{
    double dist;
    Eigen::Vector3d x;

    // calculate distance to waypoint
    x = waypoint - position;
    dist = x.norm();

    // waypoint reached?
    if (dist > threshold) {
        return 0;
    } else {
        return 1;
    }
}

int CarrotController::update(Eigen::Vector3d position, Eigen::Vector3d &carrot)
{
    int nb_waypoints;

    // pre-check
    if (this->initialized == 0) {
        return -2;
    }

    // waypoint reached? get new wp_start and wp_end
    if (this->waypointReached(position, this->wp_end, this->wp_threshold)) {
        if (this->waypoints.size() > 2) {
            this->waypoints.pop_front();
            this->wp_start = this->waypoints.at(0);
            this->wp_end = this->waypoints.at(1);
        } else {
            return 0;
        }
    }

    // calculate new carrot
    carrot = this->calculateCarrotPoint(
        position,
        this->look_ahead_dist,
        this->wp_start,
        this->wp_end
    );

    return 1;
}



// POSITION CONTROLLER
PositionController::PositionController(const std::string config_file)
{
    this->roll = 0;
    this->pitch = 0;
    this->throttle = 0;
    this->loadConfig(config_file);
}

void PositionController::loadConfig(const std::string config_file)
{
    try {
        YAML::Node config = YAML::LoadFile(config_file);

        // roll controller
        this->x.setpoint = config["roll_controller"]["setpoint"].as<float>();
        this->x.output = 0.0f;
        this->x.prev_error = 0.0f;
        this->x.sum_error = 0.0f;
        this->x.p_error = 0.0f;
        this->x.i_error = 0.0f;
        this->x.d_error = 0.0f;
        this->x.k_p = config["roll_controller"]["k_p"].as<float>();
        this->x.k_i = config["roll_controller"]["k_i"].as<float>();
        this->x.k_d = config["roll_controller"]["k_d"].as<float>();
        this->x.dead_zone = config["roll_controller"]["deadzone"].as<float>();
        this->x.min = config["roll_controller"]["min"].as<float>();
        this->x.max = config["roll_controller"]["max"].as<float>();

        // pitch controller
        this->y.setpoint = config["roll_controller"]["setpoint"].as<float>();
        this->y.output = 0.0f;
        this->y.prev_error = 0.0f;
        this->y.sum_error = 0.0f;
        this->y.p_error = 0.0f;
        this->y.i_error = 0.0f;
        this->y.d_error = 0.0f;
        this->y.k_p = config["roll_controller"]["k_p"].as<float>();
        this->y.k_i = config["roll_controller"]["k_i"].as<float>();
        this->y.k_d = config["roll_controller"]["k_d"].as<float>();
        this->y.dead_zone = config["roll_controller"]["deadzone"].as<float>();
        this->y.min = config["roll_controller"]["min"].as<float>();
        this->y.max = config["roll_controller"]["max"].as<float>();

        // throttle_controller
        this->hover_throttle = config["throttle_controller"]["hover_throttle"].as<float>();
        this->T.setpoint = config["throttle_controller"]["setpoint"].as<float>();
        this->T.output = 0.0f;
        this->T.prev_error = 0.0f;
        this->T.sum_error = 0.0f;
        this->T.p_error = 0.0f;
        this->T.i_error = 0.0f;
        this->T.d_error = 0.0f;
        this->T.k_p = config["throttle_controller"]["k_p"].as<float>();
        this->T.k_i = config["throttle_controller"]["k_i"].as<float>();
        this->T.k_d = config["throttle_controller"]["k_d"].as<float>();
        this->T.dead_zone = config["throttle_controller"]["deadzone"].as<float>();
        this->T.min = config["throttle_controller"]["min"].as<float>();
        this->T.max = config["throttle_controller"]["max"].as<float>();

    } catch (YAML::BadFile &ex) {
        throw;
    }
}

static void pid_calculate(struct pid *p, float input, float dt)
{
    float error;

    // calculate errors
    error = p->setpoint - input;
    if (fabs(error) > p->dead_zone) {
        p->sum_error += error * dt;
    }

    // calculate output
    p->p_error = p->k_p * error;
    p->i_error = p->k_i * p->sum_error;
    p->d_error = p->k_d * (error - p->prev_error) / dt;
    p->output = p->p_error + p->i_error + p->d_error;

    // limit boundaries
    if (p->output > p->max) {
        p->output = p->max;
    } else if (p->output < p->min) {
        p->output = p->min;
    }

    // update error
    p->prev_error = error;
}

void PositionController::calculate(
        Position setpoint,
        Pose robot,
        float yaw_setpoint,
        float dt,
        int global_frame
    )
{
    float roll;
    float pitch;
    float throttle;
    float roll_adjusted;
    float pitch_adjusted;
    float throttle_adjusted;

    // Note: Position Controller is (x - roll, y - pitch, T - thrust)
    // This position controller assumes yaw is aligned with the world x axis
    this->x.setpoint = -1 * setpoint.y;
    this->y.setpoint = -1 * setpoint.x;
    this->T.setpoint = setpoint.z;

    pid_calculate(&this->x, robot.y, dt);
    pid_calculate(&this->y, robot.x, dt);
    pid_calculate(&this->T, robot.z, dt);

    roll =  this->x.output;
    pitch =  this->y.output;
    throttle = this->T.output;

    if (robot.yaw < 0) {
        // make sure yaw is within 0 - 360
        robot.yaw += 2 * M_PI;
    }

    // throttle
    throttle_adjusted = this->hover_throttle + throttle;
    throttle_adjusted /= fabs(cos(roll) * cos(pitch));
    if (throttle_adjusted > 1.0) {
        throttle_adjusted = 1.0;
    }

    // update position controller
    // this->roll = roll_adjusted;
    if (global_frame == 1){
    // adjust roll and pitch according to yaw
        this->roll = cos(robot.yaw) * roll - sin(robot.yaw) * pitch;
        this->pitch = sin(robot.yaw) * roll + cos(robot.yaw) * pitch;
        this->rpy_quat = euler2quat(this->roll, this->pitch, yaw_setpoint);
    } else{

        this->roll = roll;
        this->pitch = pitch;
        this->rpy_quat = euler2quat(this->roll, this->pitch, yaw_setpoint);
    }
    this->throttle = throttle_adjusted;
}



// LANDING CONTROLLER
LandingController::LandingController(const std::string config_file)
{
    this->loadConfig(config_file);
}

void LandingController::loadConfig(const std::string config_file)
{
    try {
        YAML::Node config = YAML::LoadFile(config_file);

        // thrust controller
        this->thrust.setpoint = config["thrust_controller"]["setpoint"].as<float>();
        this->thrust.output = 0.0f;
        this->thrust.prev_error = 0.0f;
        this->thrust.sum_error = 0.0f;
        this->thrust.p_error = 0.0f;
        this->thrust.i_error = 0.0f;
        this->thrust.d_error = 0.0f;
        this->thrust.k_p = config["thrust_controller"]["k_p"].as<float>();
        this->thrust.k_i = config["thrust_controller"]["k_i"].as<float>();
        this->thrust.k_d = config["thrust_controller"]["k_d"].as<float>();
        this->thrust.dead_zone = config["thrust_controller"]["deadzone"].as<float>();
        this->thrust.min = config["thrust_controller"]["min"].as<float>();
        this->thrust.max = config["thrust_controller"]["max"].as<float>();

    } catch (YAML::BadFile &ex) {
        throw;
    }
}

void LandingController::calculate(float setpoint, Pose robot, float dt)
{


}
