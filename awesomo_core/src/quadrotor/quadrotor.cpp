#include "awesomo_core/quadrotor/quadrotor.hpp"


namespace awesomo {

Quadrotor::Quadrotor(void) {
  this->configured = false;

  this->position_controller = PositionController();
  this->tracking_controller = TrackingController();
  // this->velocity_controller = VelocityController();
  this->att_cmd = AttitudeCommand();

  this->min_discover_time = FLT_MAX;
  this->min_tracking_time = FLT_MAX;
  this->discover_tic = (struct timespec){0};
  this->tracking_tic = (struct timespec){0};

  this->current_mode = DISCOVER_MODE;
  this->heading = 0.0;
  this->pose = Pose();
  this->hover_position << 0.0, 0.0, 0.0;
  this->landing_target = LandingTarget();
}

int Quadrotor::configure(std::string config_path) {
  std::string config_file;
  ConfigParser parser;

  // position controller
  config_file = config_path + "/controllers/" + "position_controller.yaml";
  CONFIGURE_CONTROLLER(this->position_controller, config_file, FCONFPCTRL);

  // tracking controller
  config_file = config_path + "/controllers/" + "tracking_controller.yaml";
  CONFIGURE_CONTROLLER(this->tracking_controller, config_file, FCONFTCTRL);

  // velocity controller
  config_file = config_path + "/controllers/" + "velocity_controller.yaml";
  CONFIGURE_CONTROLLER(this->velocity_controller, config_file, FCONFTCTRL);

  // load config
  // clang-format off
  parser.addParam<Vec3>("hover_position", &this->hover_position);
  parser.addParam<double>("target_lost_threshold", &this->landing_target.lost_threshold);
  parser.addParam<double>("min_discover_time", &this->min_discover_time);
  parser.addParam<double>("min_track_time", &this->min_tracking_time);
  // clang-format on
  if (parser.load(config_path + "/config.yaml") != 0) {
    return -1;
  }

  this->current_mode = DISCOVER_MODE;
  this->configured = true;

  return 0;
error:
  return -1;
}

void Quadrotor::setMode(enum Mode mode) {
  this->current_mode = mode;
  switch (mode) {
    case DISARM_MODE:
      log_info(INFO_KMODE);
      break;
    case HOVER_MODE:
      log_info(INFO_HMODE);
      break;
    case DISCOVER_MODE:
      log_info(INFO_DMODE);
      break;
    case TRACKING_MODE:
      log_info(INFO_TMODE);
      break;
    case LANDING_MODE:
      log_info(INFO_LMODE);
      break;
  }
}

void Quadrotor::setPose(Pose pose) {
  this->pose = pose;
}

void Quadrotor::setVelocity(Vec3 velocity) {
  this->velocity = velocity;
}

void Quadrotor::setTargetPosition(Vec3 position, bool detected) {
  this->landing_target.setTargetPosition(position, detected);
}

void Quadrotor::setHoverXYPosition(Vec3 position) {
  this->hover_position(0) = position(0);
  this->hover_position(1) = position(1);
}

void Quadrotor::setHoverPosition(Vec3 position) {
  this->hover_position(0) = position(0);
  this->hover_position(1) = position(1);
  this->hover_position(2) = position(2);
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
    this->heading,
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
  conditions[0] = this->landing_target.target_detected;
  conditions[1] = this->landing_target.target_losted == false;
  conditions[2] = mtoc(&this->discover_tic) > this->min_discover_time;

  if (this->conditionsMet(conditions, 3)) {
    // transition to tracking mode
    this->setMode(TRACKING_MODE);
    this->discover_tic = (struct timespec){0};
  }

  return 0;
}

int Quadrotor::stepTrackingMode(double dt) {
  Vec3 errors, position;
  Vec4 tctrl_output, vctrl_output;
  bool conditions[3];

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  position = this->pose.position;
  errors(0) = this->landing_target.target_bpf(0);
  errors(1) = this->landing_target.target_bpf(1);
  errors(2) = this->hover_position(2) - position(2);

  // track target
  tctrl_output = this->tracking_controller.calculate(errors, this->heading, dt);
  // vctrl_output = this->velocity_controller.calculate(errors, dt);
  this->att_cmd = AttitudeCommand(tctrl_output);
  // this->att_cmd = AttitudeCommand(tctrl_output + vctrl_output);

  // update hover position and tracking timer
  this->setHoverXYPosition(position);
  if (this->tracking_tic.tv_sec == 0) {
    tic(&this->tracking_tic);
  }

  // transition
  conditions[0] = this->landing_target.target_detected;
  conditions[1] = this->landing_target.target_losted == false;
  conditions[2] = mtoc(&this->tracking_tic) > this->min_tracking_time;

  if (this->conditionsMet(conditions, 3)) {
    // transition to landing mode
    this->setMode(LANDING_MODE);
    this->tracking_tic = (struct timespec){0};

  } else if (this->landing_target.isTargetLosted()) {
    // transition back to discover mode
    log_info("Landing Target is losted!");
    log_info("Transitioning back to [DISCOVER_MODE]!");
    this->setMode(DISCOVER_MODE);
    this->tracking_tic = (struct timespec){0};
  }

  return 0;
}

int Quadrotor::stepLandingMode(double dt) {
  Vec3 errors, position;
  Vec4 output;
  bool conditions[3];

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  position = this->pose.position;
  errors(0) = this->landing_target.target_bpf(0);
  errors(1) = this->landing_target.target_bpf(1);
  errors(2) = this->hover_position(2) - position(2);

  // land on target
  output = this->tracking_controller.calculate(errors, this->heading, dt);
  this->att_cmd = AttitudeCommand(output);
  this->setHoverXYPosition(position);
  if (this->landing_tic.tv_sec == 0) {
    tic(&this->landing_tic);
  }

  // transition
  conditions[0] = this->landing_target.target_detected;
  conditions[1] = this->landing_target.target_losted == false;

  // if (this->conditionsMet(conditions,2)) {
  //   // transition to disarm mode
  //   this->setMode(DISARM_MODE);
  //   this->landing_tic = (struct timespec){0};
  //
  if (this->landing_target.isTargetLosted()) {
    // transition back to discovery mode
    log_info("Landing Target is losted!");
    log_info("Transitioning back to [DISCOVER_MODE]!");
    this->setMode(DISCOVER_MODE);
    this->landing_tic = (struct timespec){0};
  }

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
      return 1;
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
      log_err(EINVMODE);
      retval = this->stepHoverMode(dt);
      break;
  }
  this->landing_target.update();

  return retval;
}

}  // end of awesomo namespace
