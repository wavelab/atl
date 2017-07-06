#include "atl/quadrotor/quadrotor.hpp"


namespace atl {

int Quadrotor::configure(std::string config_path) {
  std::string config_file;
  ConfigParser parser;

  // position controller
  config_file = config_path + "/controllers/" + "position_controller.yaml";
  CONFIGURE_CONTROLLER(this->position_controller, config_file, FCONFPCTRL);

  // tracking controller
  config_file = config_path + "/controllers/" + "tracking_controller.yaml";
  CONFIGURE_CONTROLLER(this->tracking_controller, config_file, FCONFTCTRL);

  // landing controller
  config_file = config_path + "/controllers/" + "landing_controller.yaml";
  CONFIGURE_CONTROLLER(this->landing_controller, config_file, FCONFTCTRL);

  // load config
  // clang-format off
  parser.addParam("hover_position", &this->hover_position);
  parser.addParam("recover_height", &this->recover_height);
  parser.addParam("auto_track", &this->auto_track);
  parser.addParam("auto_land", &this->auto_land);
  parser.addParam("auto_disarm", &this->auto_disarm);
  parser.addParam("target_lost_threshold", &this->target_lost_threshold);
  parser.addParam("min_discover_time", &this->min_discover_time);
  parser.addParam("min_tracking_time", &this->min_tracking_time);
  // clang-format on
  if (parser.load(config_path + "/config.yaml") != 0) {
    return -1;
  }
  this->landing_target.lost_threshold = this->target_lost_threshold;
  this->current_mode = DISCOVER_MODE;
  this->configured = true;

  return 0;
error:
  return -1;
}

int Quadrotor::setMode(enum Mode mode) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set mode
  this->current_mode = mode;
  switch (mode) {
    case DISARM_MODE:
      LOG_INFO(INFO_KMODE);
      break;
    case HOVER_MODE:
      LOG_INFO(INFO_HMODE);
      break;
    case DISCOVER_MODE:
      LOG_INFO(INFO_DMODE);
      break;
    case TRACKING_MODE:
      LOG_INFO(INFO_TMODE);
      break;
    case LANDING_MODE:
      LOG_INFO(INFO_LMODE);
      break;
    default:
      LOG_ERROR(EINVMODE);
      return -2;
  }

  return 0;
}

int Quadrotor::setPose(Pose pose) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set pose
  this->pose = pose;
  return 0;
}

int Quadrotor::setVelocity(Vec3 velocity) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set pose
  this->velocity = velocity;

  return 0;
}

int Quadrotor::setYaw(double yaw) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set yaw
  this->yaw = yaw;

  return 0;
}

int Quadrotor::setTargetPosition(Vec3 position) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set target position
  this->landing_target_prev.position_bf = this->landing_target.position_bf;
  this->landing_target.setTargetPosition(position);

  return 0;
}

int Quadrotor::setTargetVelocity(Vec3 velocity) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set target velocity
  this->landing_target_prev.velocity_bf = this->landing_target.velocity_bf;
  this->landing_target.setTargetVelocity(velocity);

  return 0;
}

int Quadrotor::setTargetDetected(bool detected) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set target detected
  this->landing_target.update(detected);

  return 0;
}


int Quadrotor::setHoverXYPosition(Vec3 position) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set hover x-y position
  this->hover_position(0) = position(0);
  this->hover_position(1) = position(1);

  return 0;
}

int Quadrotor::setHoverPosition(Vec3 position) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // set hover position
  this->hover_position(0) = position(0);
  this->hover_position(1) = position(1);
  this->hover_position(2) = position(2);

  return 0;
}

bool Quadrotor::conditionsMet(bool *conditions, int nb_conditions) {
  for (int i = 0; i < nb_conditions; i++) {
    if (conditions[i] == false) {
      return false;
    }
  }

  return true;
}

int Quadrotor::stepHoverMode(double dt) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // hover
  // clang-format off
  this->position_controller.calculate(
    this->hover_position,
    this->pose,
    this->yaw,
    dt
  );
  // clang-format on
  this->att_cmd = AttitudeCommand(this->position_controller.outputs);

  return 0;
}

int Quadrotor::stepDiscoverMode(double dt) {
  bool conditions[3];

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // hover in place
  this->stepHoverMode(dt);
  if (this->discover_tic.tv_sec == 0) {
    tic(&this->discover_tic);
  }

  // transition
  conditions[0] = this->landing_target.detected;
  conditions[1] = this->landing_target.losted == false;
  conditions[2] = mtoc(&this->discover_tic) > this->min_discover_time;

  if (this->conditionsMet(conditions, 3) && this->auto_track) {
    // transition to tracking mode
    LOG_INFO("Transitioning to [TRACKING MODE]!");
    this->setMode(TRACKING_MODE);
    this->discover_tic = (struct timespec){0, 0};
  }

  return 0;
}

int Quadrotor::stepTrackingMode(double dt) {
  int retval;
  bool conditions[3];
  Quaternion q;
  Vec3 euler, vel_bf;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // calculate velocity in body frame
  euler << 0, 0, this->yaw;
  euler2quat(euler, 321, q);
  inertial2body(this->velocity, q, vel_bf);

  // track target
  this->att_cmd =
    this->tracking_controller.calculate(this->landing_target.position_bf,
                                        this->pose.position,
                                        this->hover_position,
                                        this->yaw,
                                        dt);

  // update hover position and tracking timer
  this->setHoverXYPosition(this->pose.position);
  if (this->tracking_tic.tv_sec == 0) {
    tic(&this->tracking_tic);
  }

  // transition
  conditions[0] = this->landing_target.detected;
  conditions[1] = this->landing_target.losted == false;
  conditions[2] = mtoc(&this->tracking_tic) > this->min_tracking_time;

  // check conditions
  if (this->conditionsMet(conditions, 3) && this->auto_land) {
    // load trajectory
    retval = this->landing_controller.loadTrajectory(
      this->pose.position, this->landing_target.position_bf, vel_bf(0));

    // transition to landing mode
    if (retval == 0) {
      LOG_INFO("Transitioning to [LANDING MODE]!");
      // this->landing_controller.recordTrajectoryIndex();
      this->setMode(LANDING_MODE);
      this->tracking_tic = (struct timespec){0, 0};
    }

  } else if (this->landing_target.isTargetLosted()) {
    // transition back to discover mode
    LOG_INFO("Landing Target is losted!");
    LOG_INFO("Transitioning back to [DISCOVER_MODE]!");
    this->setMode(DISCOVER_MODE);
    this->tracking_tic = (struct timespec){0, 0};
    this->hover_position(2) = this->recover_height;
  }

  return 0;
}

int Quadrotor::stepLandingMode(double dt) {
  int retval;
  bool conditions[3];

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // land on target
  retval =
    this->landing_controller.calculate(this->landing_target.position_bf,
                                       this->landing_target.velocity_bf,
                                       this->pose.position,
                                       this->velocity,
                                       this->pose.orientation,
                                       this->yaw,
                                       dt);
  this->att_cmd = this->landing_controller.att_cmd;

  // update hover position and tracking timer
  this->setHoverPosition(this->pose.position);
  if (this->landing_tic.tv_sec == 0) {
    tic(&this->landing_tic);
  }

  // transition
  conditions[0] = this->landing_target.detected;
  conditions[1] = this->landing_target.losted == false;
  conditions[2] = this->landing_target.position_bf.norm() < 0.1;

  // check conditions
  if (this->conditionsMet(conditions, 3) && this->auto_disarm) {
    // transition to disarm mode
    this->setMode(DISARM_MODE);
    this->landing_tic = (struct timespec){0, 0};

  } else if (this->landing_target.isTargetLosted()) {
    // transition back to discovery mode
    LOG_INFO("Landing Target is losted!");
    LOG_INFO("Transitioning back to [DISCOVER_MODE]!");
    this->setMode(DISCOVER_MODE);
    this->landing_tic = (struct timespec){0, 0};
    this->hover_position(2) = this->recover_height;

  } else if (retval != 0) {
    // transition back to discovery mode
    LOG_INFO("Failed to follow landing trajectory!");
    LOG_INFO("Transitioning back to [DISCOVER_MODE]!");
    this->setMode(DISCOVER_MODE);
    this->landing_tic = (struct timespec){0, 0};
    this->hover_position(2) = this->recover_height;
  }

  return 0;
}

int Quadrotor::reset(void) {
  this->position_controller.reset();
  this->tracking_controller.reset();
  this->landing_controller.reset();

  return 0;
}

int Quadrotor::step(double dt) {
  int retval;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // step
  switch (this->current_mode) {
    case DISARM_MODE:
      this->att_cmd = AttitudeCommand();
      retval = 0;
      break;
    case HOVER_MODE:
      retval = this->stepHoverMode(dt);
      break;
    case DISCOVER_MODE:
      retval = this->stepDiscoverMode(dt);
      break;
    case TRACKING_MODE:
      retval = this->stepTrackingMode(dt);
      break;
    case LANDING_MODE:
      retval = this->stepLandingMode(dt);
      break;
    default:
      LOG_ERROR(EINVMODE);
      retval = this->stepHoverMode(dt);
      break;
  }

  return retval;
}

}  // namespace atl
