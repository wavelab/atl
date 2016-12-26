#include "awesomo_core/vision/camera/camera.hpp"


namespace awesomo {

Camera::Camera(void) {
  this->configured = false;
  this->initialized = false;

  this->current_config = NULL;
  this->modes.clear();
  this->configs.clear();

  this->capture = NULL;
  this->last_tic = 0.0;
}

Camera::~Camera(void) {
  CameraConfig *config;

  if (this->configured) {
    for (int i = 0; i < this->configs.size(); i++) {
      delete this->configs[this->modes[i]];
    }
  }

  if (this->initialized) {
    this->capture->release();
    this->capture = NULL;
  }
}

int Camera::configure(std::string config_path) {
  ConfigParser parser;
  CameraConfig *config;
  std::vector<std::string> camera_modes;
  std::vector<std::string> camera_configs;

  // load config
  parser.addParam(STRING_ARRAY, "modes", &camera_modes);
  parser.addParam(STRING_ARRAY, "configs", &camera_configs);
  if (parser.load(config_path + "/" + "config.yaml") != 0) {
    log_err("Failed to configure camera!");
    return -1;
  }

  // load camera configs
  for (int i = 0; i < camera_modes.size(); i++) {
    config = new CameraConfig();
    if (config->load(config_path + "/" + camera_configs[i]) != 0) {
      log_err("Failed to configure camera!");
      return -1;
    }

    this->modes.push_back(camera_modes[i]);
    this->configs[camera_modes[i]] = config;
  }
  this->current_config = this->configs[camera_modes[0]];
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
  camera_index = this->current_config->index;
  image_width = this->current_config->image_width;
  image_height = this->current_config->image_height;

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
  if (this->initialized) {
    this->capture->release();
    this->capture = NULL;
    this->initialized = false;
  }
}

int Camera::getFrame(cv::Mat &image) {
  // pre-check
  if (this->configured == false) {
    return -1;
  } else if (this->initialized == false) {
    return -2;
  }

  this->capture->read(image);
  return 0;
}

int Camera::run(void) {
  cv::Mat image;
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
    this->getFrame(image);

    // show stats
    this->showFPS(last_tic, frame_count);
    this->showImage(image);
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
  if (frame_count % 30 == 0 && this->current_config->showfps) {
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
  if (this->current_config->imshow) {
    cv::imshow("Camera", image);
    cv::waitKey(1);
  }
}

}  // end of awesomo namespace
