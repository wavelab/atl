#include "atl/vision/camera/dc1394.hpp"

namespace atl {

int DC1394Camera::initialize(uint64_t guid) {
  dc1394error_t error;

  // connect to DC1394
  this->dc1394 = dc1394_new();
  if (!this->dc1394) {
    LOG_ERROR("Failed to connect to DC1394");
  }

  // enumerate camera
  dc1394camera_list_t *list;
  error = dc1394_camera_enumerate(this->dc1394, &list);
  DC1394_ERR_RTN(error, "Failed to enumerate cameras");
  if (list->num == 0) {
    dc1394_log_error("No cameras found");
    return -1;
  } else {
    std::cout << std::endl;
    std::cout << "Found " << list->num << " camera(s)" << std::endl;
    std::cout << "----------" << std::endl;
    for (size_t i = 0; i < list->num; i++) {
      std::cout << "camera[" << i << "] - guid: ";
      std::cout << list->ids[i].guid << std::endl;
    }
    std::cout << std::endl;
  }

  // connect to camera
  guid = (guid == 0) ? list->ids[0].guid : guid;
  this->capture = dc1394_camera_new(this->dc1394, guid);
  if (!this->capture) {
    dc1394_log_error("Failed to connect to camera with guid %" PRIu64, guid);
    return -1;
  }
  LOG_INFO("Connected to camera with guid %" PRIu64, guid);
  dc1394_camera_free_list(list);

  // operation mode - FireWire IEEE 1394a or 1394b
  error = dc1394_video_set_operation_mode(this->capture,
                                          DC1394_OPERATION_MODE_LEGACY);
  DC1394_ERR_RTN(error, "Failed to set 1394A mode!");

  // capture settings
  error = dc1394_capture_setup(this->capture, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
  DC1394_ERR_RTN(error, "Failed to configure camera!");

  // video mode
  error = dc1394_video_set_mode(this->capture, DC1394_VIDEO_MODE_640x480_MONO8);
  DC1394_ERR_RTN(error, "Failed to set video mode!");

  // iso speed
  error = dc1394_video_set_iso_speed(this->capture, DC1394_ISO_SPEED_400);
  DC1394_ERR_RTN(error, "Failed to set iso speed!");

  // frame rate
  this->setFrameRate(60);

  // exposure
  this->setExposure(this->config.exposure_value);

  // shutter
  this->setShutter(this->config.shutter_speed);

  // gain
  this->setGain(this->config.gain_value);

  // start transmission
  error = dc1394_video_set_transmission(this->capture, DC1394_ON);
  DC1394_ERR_RTN(error, "Failed to start camera transmission!");

  // update
  LOG_INFO("Camera initialized!");
  this->initialized = true;

  return 0;
}

void DC1394Camera::printFrameInfo(dc1394video_frame_t *frame) {
  // clang-format off
  std::cout << "Frame Info:" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
  std::cout << "image_bytes: %d" << frame->image_bytes << std::endl;
  std::cout << "size[0]: %d" << frame->size[0] << std::endl;
  std::cout << "size[1]: %d" << frame->size[1] << std::endl;
  std::cout << "allocated_image_bytes: %d" << frame->allocated_image_bytes;
  std::cout << "total_bytes: %d" << frame->total_bytes << std::endl;
  std::cout << "color_coding: %d" << frame->color_coding << std::endl;
  std::cout << "color_filter: %d" << frame->color_filter << std::endl;
  std::cout << "packet_size: %d" << frame->packet_size << std::endl;
  std::cout << "packets_per_frame: %d" << frame->packets_per_frame << std::endl;
  std::cout << "padding_bytes: %d" << frame->padding_bytes << std::endl;
  std::cout << "timestamp: %d" << frame->timestamp << std::endl;
  std::cout << "stride: %d" << frame->stride << std::endl;
  std::cout << "data_depth: %d" << frame->data_depth << std::endl;
  std::cout << "id: %d" << frame->id << std::endl;
  std::cout << "frames_behind: %d" << frame->frames_behind << std::endl;
  // clang-format on
}

int DC1394Camera::setBrightness(double brightness) {
  dc1394error_t err;

  // turn on the feature - dont know what this means??
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on the brightness feature");

  // turn off auto exposure
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_BRIGHTNESS,
                                DC1394_FEATURE_MODE_MANUAL);
  DC1394_ERR_RTN(err, "Could not turn off Auto-brightness");

  // set brightness
  err = dc1394_feature_set_value(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 brightness);
  DC1394_ERR_RTN(err, "Could not set value");

  return 0;
}

int DC1394Camera::setFrameRate(double fps) {
  dc1394framerate_t framerate;

  // convert fps to dc1394 framerate type
  if (fltcmp(fps, 1.875) == 0) {
    framerate = DC1394_FRAMERATE_1_875;
  } else if (fltcmp(fps, 3.75) == 0) {
    framerate = DC1394_FRAMERATE_3_75;
  } else if (fltcmp(fps, 7.5) == 0) {
    framerate = DC1394_FRAMERATE_7_5;
  } else if (fltcmp(fps, 15.0) == 0) {
    framerate = DC1394_FRAMERATE_15;
  } else if (fltcmp(fps, 30.0) == 0) {
    framerate = DC1394_FRAMERATE_30;
  } else if (fltcmp(fps, 60.0) == 0) {
    framerate = DC1394_FRAMERATE_60;
  } else if (fltcmp(fps, 120.0) == 0) {
    framerate = DC1394_FRAMERATE_120;
  } else if (fltcmp(fps, 240.0) == 0) {
    framerate = DC1394_FRAMERATE_240;
  } else {
    LOG_ERROR("Unsupported or invalid frame rate %f", fps);
    return -1;
  }

  // set camera frame rate
  dc1394error_t err = dc1394_video_set_framerate(this->capture, framerate);
  DC1394_ERR_RTN(err, "Failed to set frame rate!");

  return 0;
}

int DC1394Camera::setExposure(double exposure) {
  dc1394error_t err;

  // turn on the exposure feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_EXPOSURE,
                                 DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on the exposure feature");

  // turn off auto exposure
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_EXPOSURE,
                                DC1394_FEATURE_MODE_MANUAL);
  DC1394_ERR_RTN(err, "Could not turn off Auto-exposure");

  // set exposure
  err = dc1394_feature_set_value(this->capture,
                                 DC1394_FEATURE_EXPOSURE,
                                 exposure);
  DC1394_ERR_RTN(err, "Could not set exposure");

  return 0;
}

int DC1394Camera::setShutter(double shutter_ms) {
  dc1394error_t err;

  // turn on the shutter feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_SHUTTER,
                                 DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on the shutter feature");

  // turn off auto shutter
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_SHUTTER,
                                DC1394_FEATURE_MODE_MANUAL);
  DC1394_ERR_RTN(err, "Could not turn off Auto-shutter");

  // set shutter
  err = dc1394_feature_set_value(this->capture,
                                 DC1394_FEATURE_SHUTTER,
                                 shutter_ms);
  DC1394_ERR_RTN(err, "Could not set shutter");

  return 0;
}

int DC1394Camera::setGain(double gain_db) {
  dc1394error_t err;

  // turn on the gain feature
  err = dc1394_feature_set_power(this->capture, DC1394_FEATURE_GAIN, DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on the gain feature");

  // turn off auto gain
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_GAIN,
                                DC1394_FEATURE_MODE_MANUAL);
  DC1394_ERR_RTN(err, "Could not turn off Auto-gain");

  // set gain
  err = dc1394_feature_set_value(this->capture, DC1394_FEATURE_GAIN, gain_db);
  DC1394_ERR_RTN(err, "Could not set gain");

  return 0;
}

int DC1394Camera::getBrightness(double &brightness) {
  dc1394error_t err;

  // turn on the brightness feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on brightness feature");

  // get brightness value
  uint32_t value;
  err = dc1394_feature_get_value(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 &value);
  DC1394_ERR_RTN(err, "Could not get brightness");
  brightness = (double) value;

  return 0;
}

int DC1394Camera::getFrameRate(double &fps) {
  dc1394framerate_t framerate;

  // get camera frame rate
  dc1394error_t err = dc1394_video_get_framerate(this->capture, &framerate);
  DC1394_ERR_RTN(err, "Failed to get frame rate!");

  // convert fps to dc1394 framerate type
  if (framerate == DC1394_FRAMERATE_1_875) {
    fps = 1.875;
  } else if (framerate == DC1394_FRAMERATE_3_75) {
    fps = 3.75;
  } else if (framerate == DC1394_FRAMERATE_7_5) {
    fps = 7.5;
  } else if (framerate == DC1394_FRAMERATE_15) {
    fps = 15.0;
  } else if (framerate == DC1394_FRAMERATE_30) {
    fps = 30.0;
  } else if (framerate == DC1394_FRAMERATE_60) {
    fps = 60.0;
  } else if (framerate == DC1394_FRAMERATE_120) {
    fps = 120.0;
  } else if (framerate == DC1394_FRAMERATE_240) {
    fps = 240.0;
  } else {
    LOG_ERROR("Unsupported or invalid frame rate %f", fps);
    return -1;
  }

  return 0;
}

int DC1394Camera::getExposure(double &exposure) {
  dc1394error_t error;

  // turn on the exposure feature
  error = dc1394_feature_set_power(this->capture,
                                   DC1394_FEATURE_EXPOSURE,
                                   DC1394_ON);
  DC1394_ERR_RTN(error, "Could not turn on exposure feature");

  // get exposure value
  uint32_t value;
  error =
      dc1394_feature_get_value(this->capture, DC1394_FEATURE_EXPOSURE, &value);
  DC1394_ERR_RTN(error, "Could not get exposure");
  exposure = (double) value;

  return 0;
}

int DC1394Camera::getShutter(double &shutter_ms) {
  dc1394error_t err;

  // turn on the exposure feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_SHUTTER,
                                 DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on shutter feature");

  // get shutter value
  uint32_t value;
  err = dc1394_feature_get_value(this->capture, DC1394_FEATURE_SHUTTER, &value);
  DC1394_ERR_RTN(err, "Could not get shutter");
  shutter_ms = (double) value;

  return 0;
}

int DC1394Camera::getGain(double &gain_db) {
  dc1394error_t err;

  // turn on the gain feature
  err = dc1394_feature_set_power(this->capture, DC1394_FEATURE_GAIN, DC1394_ON);
  DC1394_ERR_RTN(err, "Could not turn on gain feature");

  // get shutter value
  uint32_t value;
  err = dc1394_feature_get_value(this->capture, DC1394_FEATURE_GAIN, &value);
  DC1394_ERR_RTN(err, "Could not get gain");
  gain_db = (double) value;

  return 0;
}

std::pair<int, int> DC1394Camera::centerROI(const int size,
                                            const int max_size,
                                            const int step) {
  double roi_size;
  if (size == 0 || size > max_size) {
    roi_size = max_size;
  }

  // size must be a multiple of the step
  roi_size = size / step * step;
  int offset = (max_size - roi_size) / 2;
  return std::make_pair(roi_size, offset);
}

int DC1394Camera::changeMode(const std::string &mode) {
  // pre-check
  if (this->configs.find(mode) == this->configs.end()) {
    return -1;
  }

  // update camera settings
  this->config = this->configs[mode];

  return 0;
}

int DC1394Camera::getFrame(cv::Mat &image) {
  dc1394error_t err;

  // capture frame
  dc1394video_frame_t *frame = nullptr;
  err =
      dc1394_capture_dequeue(this->capture, DC1394_CAPTURE_POLICY_WAIT, &frame);
  DC1394_ERR_RTN(err, "Failed to obtain frame from camera!");

  // convert mono 8 to colour
  size_t channels = 3;
  size_t img_size = frame->image_bytes * channels;
  size_t img_rows = frame->size[1];
  size_t img_cols = frame->size[0];
  size_t row_bytes = img_size / img_rows;

  if (this->buffer == nullptr) {
    buffer = (uint8_t *) malloc(img_size);
  }
  dc1394_bayer_decoding_8bit(frame->image,
                             this->buffer,
                             img_cols,
                             img_rows,
                             DC1394_COLOR_FILTER_BGGR,
                             DC1394_BAYER_METHOD_SIMPLE);

  // convert to opencv mat
  cv::Mat(img_rows, img_cols, CV_8UC3, this->buffer, row_bytes).copyTo(image);

  // release frame
  err = dc1394_capture_enqueue(this->capture, frame);
  DC1394_ERR_RTN(err, "Failed to release frame buffer!");

  return 0;
}

int DC1394Camera::run() {
  int frame_count;
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

} // namespace atr
