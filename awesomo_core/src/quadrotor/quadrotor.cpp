#include "awesomo_core/quadrotor/quadrotor.hpp"


namespace awesomo {

Quadrotor::Quadrotor(void) {
  this->configured = false;

  this->current_mode = HOVER_MODE;

  this->heading = 0.0;
  this->pose = Pose();
  this->hover_position << 0.0, 0.0, 0.0;
  this->target_losted = true;
  this->target_detected = false;
  this->target_bpf << 0.0, 0.0, 0.0;

  this->position_controller = PositionController();
  this->att_cmd = AttitudeCommand();
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

  // load config
  parser.addParam<Vec3>("hover_position", &this->hover_position);
  if (parser.load(config_path + "/config.yaml") != 0) {
    return -1;
  }

  this->current_mode = HOVER_MODE;
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

void Quadrotor::setTargetPosition(Vec3 position, bool detected) {
  this->target_detected = detected;
  this->target_bpf = position;
}

int Quadrotor::stepHoverMode(double dt) {
  Vec3 euler, setpoint, nwu;
  Vec4 actual, output;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  enu2nwu(this->hover_position, setpoint);

  // transform ENU to NWU
  quat2euler(this->pose.q, 321, euler);
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
  // clang-format on

  return 0;
}

// int Quadrotor::stepDiscoverMode(double dt) {
//   enum Mode new_mode;
//
//   // pre-check
//   if (this->configured == false) {
//     return -1;
//   }
//
//   // update
//   this->stepHoverMode(dt);
//   this->discover_mode.update();
//
//   // transition
//   if (this->discover_mode.transition(new_mode)) {
//     this->setMode(new_mode);
//     this->tracking_mode.target_bpf = this->discover_mode.target_bpf;
//     this->hover_mode.stop();
//   }
//
//   return 0;
// }
//
// int Quadrotor::stepTrackingMode(double dt) {
//   enum Mode new_mode;
//   Vec3 errors, position;
//   Vec4 output;
//
//   // pre-check
//   if (this->configured == false) {
//     return -1;
//   }
//
//   // setup
//   position = this->pose.position;
//   errors(0) = this->tracking_mode.target_bpf(0);
//   errors(1) = this->tracking_mode.target_bpf(1);
//   errors(2) = this->hover_mode.hover_height - position(2);
//
//   // track target
//   output = this->tracking_controller.calculate(errors, this->heading, dt);
//   this->att_cmd = AttitudeCommand(output);
//   this->hover_mode.updateHoverXYPosition(position(0), position(1));
//
//   // transition
//   if (this->tracking_mode.transition(new_mode)) {
//     this->setMode(new_mode);
//   }
//
//   return 0;
// }

// int Quadrotor::stepLandingMode(double dt) {
//   enum Mode new_mode;
//   Vec3 errors, position;
//   Vec4 output;
//
//   // pre-check
//   if (this->configured == false) {
//     return -1;
//   }
//
//   // setup
//   position = this->pose.position;
//   errors(0) = this->tracking_mode.target_bpf(0);
//   errors(1) = this->tracking_mode.target_bpf(1);
//   errors(2) = this->hover_mode.hover_height - position(2);
//
//   // track target
//   output = this->tracking_controller.calculate(errors, this->heading, dt);
//   this->att_cmd = AttitudeCommand(output);
//   this->hover_mode.updateHoverXYPosition(position(0), position(1));
//
//   // transition
//   if (this->tracking_mode.transition(new_mode)) {
//     this->setMode(new_mode);
//   }
//
//   return 0;
// }

int Quadrotor::step(double dt) {
  int retval;
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // step
  switch (this->current_mode) {
    case HOVER_MODE:
      retval = this->stepHoverMode(dt);
      break;
    default:
      retval = log_err(EINVMODE);
      return -1;
  }

  return retval;
}

}  // end of awesomo namespace
