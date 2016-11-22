#include "awesomo/controller.hpp"


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
    // p->d_error = p->k_d * (error - p->prev_error) / dt;
    p->d_error = p->k_d * (error - p->prev_error) / (1.0 / 100.0);
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
    Eigen::Vector3d setpoint,
    Pose robot,
    float yaw_setpoint,
    float dt
)
{
    float roll_adjusted;
    float pitch_adjusted;
    float throttle_adjusted;

    // Note: Position Controller is (x - roll, y - pitch, T - thrust)
    // This position controller assumes yaw is aligned with the world x axis
    this->x.setpoint = setpoint(1);
    this->y.setpoint = setpoint(0);
    this->T.setpoint = setpoint(2);

    pid_calculate(&this->x, robot.position(1), dt);
    pid_calculate(&this->y, robot.position(0), dt);
    pid_calculate(&this->T, robot.position(2), dt);

    this->roll = this->x.output;
    this->pitch = this->y.output;
    this->throttle = this->T.output;

    // update position controller
    euler2Quaternion(this->roll, this->pitch, yaw_setpoint, this->command_quat);

    // throttle
    throttle_adjusted = this->hover_throttle + this->throttle;
    throttle_adjusted /= fabs(cos(roll) * cos(pitch)); // adjust due to tilting
    if (throttle_adjusted > 1.0) {
        throttle_adjusted = 1.0;
    }
    this->throttle = throttle_adjusted;
}

void PositionController::reset(void)
{
    this->roll = 0;
    this->pitch = 0;
    this->throttle = 0;

    this->x.output = 0.0f;
    this->x.prev_error = 0.0f;
    this->x.sum_error = 0.0f;
    this->x.p_error = 0.0f;
    this->x.i_error = 0.0f;
    this->x.d_error = 0.0f;

    this->y.output = 0.0f;
    this->y.prev_error = 0.0f;
    this->y.sum_error = 0.0f;
    this->y.p_error = 0.0f;
    this->y.i_error = 0.0f;
    this->y.d_error = 0.0f;

    this->T.output = 0.0f;
    this->T.prev_error = 0.0f;
    this->T.sum_error = 0.0f;
    this->T.p_error = 0.0f;
    this->T.i_error = 0.0f;
    this->T.d_error = 0.0f;
}
