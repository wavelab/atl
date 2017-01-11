#include "awesomo_core/quadrotor/quadrotor.hpp"


namespace awesomo {

Quadrotor::Quadrotor(void) {
  this->configured = false;

  this->current_mode = DISCOVER_MODE;
  this->hover_mode = HoverMode();
  this->discover_mode = DiscoverMode();
  this->tracking_mode = TrackingMode();

  this->heading = 0.0;
  this->pose = Pose();

  this->position_controller = PositionController();
  this->att_cmd = AttitudeCommand();
}

int Quadrotor::configure(std::string config_path) {
  std::string config_file;

  // position controller
  config_file = config_path + "/controllers/" + "position_controller.yaml";
  CONFIGURE_CONTROLLER(this->position_controller, config_file, FCONFPCTRL);

  // tracking controller
  config_file = config_path + "/controllers/" + "tracking_controller.yaml";
  CONFIGURE_CONTROLLER(this->tracking_controller, config_file, FCONFTCTRL);

  // hover mode
  config_file = config_path + "/modes/" + "hover_mode.yaml";
  CONFIGURE_MODE(this->hover_mode, config_file, FCONFHMODE);

  // discover mode
  config_file = config_path + "/modes/" + "discover_mode.yaml";
  CONFIGURE_MODE(this->discover_mode, config_file, FCONFDMODE);

  // tracking mode
  config_file = config_path + "/modes/" + "tracking_mode.yaml";
  CONFIGURE_MODE(this->tracking_mode, config_file, FCONFHMODE);

  this->current_mode = DISCOVER_MODE;
  this->configured = true;

  return 0;
error:
  return -1;
}

void Quadrotor::setMode(enum Mode mode) {
  this->current_mode = mode;
  switch (mode) {
    case DISARM_MODE: log_info(INFO_KMODE); break;
    case HOVER_MODE: log_info(INFO_HMODE); break;
    case DISCOVER_MODE: log_info(INFO_DMODE); break;
    case TRACKING_MODE: log_info(INFO_TMODE); break;
    case LANDING_MODE: log_info(INFO_LMODE); break;
  }
}

void Quadrotor::setPose(Pose pose) {
  this->pose = pose;
}

void Quadrotor::setTargetPosition(Vec3 position, bool detected) {
  switch (this->current_mode) {
    case DISCOVER_MODE:
       this->discover_mode.updateTargetPosition(position, detected);
       break;
    case TRACKING_MODE:
       this->tracking_mode.updateTargetPosition(position, detected);
       break;
    default:
      break;
  }
}

int Quadrotor::stepHoverMode(double dt) {
  Vec3 euler, setpoint, nwu;
  Vec4 actual, output;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  setpoint = this->hover_mode.getHoverPosition();
  quat2euler(this->pose.q, 321, euler);

  // transform ENU to NWU
  enu2nwu(this->pose.position, nwu);
  actual(0) = nwu(0);
  actual(1) = nwu(1);
  actual(2) = nwu(2);
  actual(3) = euler(2);  // yaw

  // step
  // clang-format off
  output = this->position_controller.calculate(
    setpoint,
    actual,
    this->heading,
    dt
  );
  this->att_cmd = AttitudeCommand(output);
  this->hover_mode.update();
  // clang-format on

  return 0;
}

int Quadrotor::stepDiscoverMode(double dt) {
  enum Mode new_mode;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // update
  this->stepHoverMode(dt);
  this->discover_mode.update();

  // transition
  if (this->discover_mode.transition(new_mode)) {
    this->setMode(new_mode);
    this->tracking_mode.target_bpf = this->discover_mode.target_bpf;
    this->hover_mode.stop();
  }

  return 0;
}

int Quadrotor::stepTrackingMode(double dt) {
  enum Mode new_mode;
  Vec3 errors, position;
  Vec4 output;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  position = this->pose.position;
  errors(0) = this->tracking_mode.target_bpf(0);
  errors(1) = this->tracking_mode.target_bpf(1);
  errors(2) = this->hover_mode.hover_height - position(2);

  // track target
  output = this->tracking_controller.calculate(errors, this->heading, dt);
  this->att_cmd = AttitudeCommand(output);
  this->hover_mode.updateHoverXYPosition(position(0), position(1));

  // transition
  if (this->tracking_mode.transition(new_mode)) {
    this->setMode(new_mode);
  }

  return 0;
}

int Quadrotor::stepLandingMode(double dt) {
  enum Mode new_mode;
  Vec3 errors, position;
  Vec4 output;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  position = this->pose.position;
  errors(0) = this->tracking_mode.target_bpf(0);
  errors(1) = this->tracking_mode.target_bpf(1);
  errors(2) = this->hover_mode.hover_height - position(2);

  // track target
  output = this->tracking_controller.calculate(errors, this->heading, dt);
  this->att_cmd = AttitudeCommand(output);
  this->hover_mode.updateHoverXYPosition(position(0), position(1));

  // transition
  if (this->tracking_mode.transition(new_mode)) {
    this->setMode(new_mode);
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
  // clang-format off
  switch (this->current_mode) {
    case HOVER_MODE: retval = this->stepHoverMode(dt); break;
    case DISCOVER_MODE: retval = this->stepDiscoverMode(dt); break;
    case TRACKING_MODE: retval = this->stepTrackingMode(dt); break;
    case LANDING_MODE: retval = this->stepLandingMode(dt); break;
    case DISARM_MODE: return -1;
    default: retval = log_err(EINVMODE); return -1;
  }
  // clang-format on

  return retval;
}

}  // end of awesomo namespace
