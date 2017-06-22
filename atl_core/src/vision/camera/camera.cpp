#include "atl_core/vision/camera/camera.hpp"


namespace atl {

Camera::Camera(void) {
  this->configured = false;
  this->initialized = false;

  this->config = CameraConfig();
  this->modes.clear();
  this->configs.clear();

  this->image = cv::Mat(0, 0, CV_64F);
  this->last_tic = 0.0;

  this->capture = NULL;
}

Camera::~Camera(void) {
  if (this->initialized && this->capture) {
    this->capture->release();
    this->capture = NULL;
  }
}

int Camera::configure(std::string config_path) {
  ConfigParser parser;
  CameraConfig config;
  std::string config_file;
  std::vector<std::string> camera_modes;
  std::vector<std::string> camera_configs;

  // load config
  config_file = config_path + "/" + "config.yaml";
  parser.addParam<std::vector<std::string>>("modes", &camera_modes);
  parser.addParam<std::vector<std::string>>("configs", &camera_configs);
  if (parser.load(config_file) != 0) {
    log_err("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // load camera configs
  for (int i = 0; i < camera_modes.size(); i++) {
    config = CameraConfig();
    config_file = config_path + "/" + camera_configs[i];
    if (config.load(config_file) != 0) {
      log_err("Failed to load config file [%s]!", config_file.c_str());
      return -1;
    }

    this->modes.push_back(camera_modes[i]);
    this->configs[camera_modes[i]] = config;
  }
  this->config = this->configs[camera_modes[0]];
  this->configured = true;

  return 0;
}

int Camera::initialize(void) {
  int camera_index;
  int image_width;
  int image_height;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // setup
  camera_index = this->config.index;
  image_width = this->config.image_width;
  image_height = this->config.image_height;

  // open
  this->capture = new cv::VideoCapture(camera_index);
  if (this->capture->isOpened() == 0) {
    log_err("Failed to open camera!\n");
    return -1;

  } else {
    this->capture->set(CV_CAP_PROP_FRAME_WIDTH, image_width);
    this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, image_height);
    this->initialized = true;
    log_info("Camera initialized!\n");
    return 0;
  }
}

int Camera::shutdown(void) {
  if (this->initialized && this->capture) {
    this->capture->release();
    this->capture = NULL;
    this->initialized = false;
  }
}

int Camera::changeMode(std::string mode) {
  // pre-check
  if (this->configs.find(mode) == this->configs.end()) {
    return -1;
  }

  // update camera settings
  this->config = this->configs[mode];
  this->capture->set(CV_CAP_PROP_FRAME_WIDTH, this->config.image_width);
  this->capture->set(CV_CAP_PROP_FRAME_HEIGHT, this->config.image_height);
}

int Camera::getFrame(cv::Mat &image) {
  bool change_mode;

  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->initialized == false) {
    return -2;
  }

  // get frame
  this->capture->read(image);

  return 0;
}

int Camera::run(void) {
  int frame_count;
  double t;
  double last_tic;

  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->initialized == false) {
    return -2;
  }

  // setup
  frame_count = 0;
  last_tic = time_now();

  // run
  while (true) {
    this->getFrame(this->image);

    // show stats
    this->showFPS(last_tic, frame_count);
    this->showImage(this->image);
  }

  return 0;
}

int Camera::showFPS(double &last_tic, int &frame_count) {
  double t;
  double fps;

  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->initialized == false) {
    return -2;
  }

  frame_count++;
  if (frame_count % 30 == 0 && this->config.showfps) {
    t = time_now();
    fps = 30.0 / (t - last_tic);
    printf("fps: %.2f\n", fps);
    last_tic = t;
    frame_count = 0;
  }
}

int Camera::showImage(cv::Mat &image) {
  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->initialized == false) {
    return -2;
  }

  // show image
  if (this->config.imshow && image.rows && image.cols) {
    cv::imshow("Camera", image);
    cv::waitKey(1);
  }
}

}  // end of atl namespace
