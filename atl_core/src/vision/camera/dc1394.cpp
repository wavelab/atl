#include "atl/vision/camera/dc1394.hpp"

namespace atl {

int DC1394Camera::connect(uint64_t guid) {
  dc1394error_t error;

  // connect to DC1394
  this->dc1394 = dc1394_new();
  if (!this->dc1394) {
    LOG_ERROR("Failed to initialize DC1394 interface!");
    return -1;
  }

  // enumerate camera
  dc1394camera_list_t *list;
  error = dc1394_camera_enumerate(this->dc1394, &list);
  if (error != DC1394_SUCCESS) {
    LOG_ERROR("Failed to enumerate cameras!");
    return -1;

  } else if (list->num == 0) {
    LOG_ERROR("No cameras found!");
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
    LOG_ERROR("Failed to connect to camera with guid %" PRIu64, guid);
    return -1;
  }
  LOG_INFO("Connected to camera with guid %" PRIu64, guid);
  dc1394_camera_free_list(list);

  return 0;
}

int DC1394Camera::initialize(uint64_t guid) {
  dc1394error_t error;

  // connect
  if (this->connect(guid) != 0) {
    return -1;
  }

  // operation mode - FireWire IEEE 1394a or 1394b
  error = dc1394_video_set_operation_mode(this->capture,
                                          DC1394_OPERATION_MODE_LEGACY);
  if (error != DC1394_SUCCESS) {
    LOG_ERROR("Failed to set 1394A mode!");
    return -1;
  }

  // capture settings
  error = dc1394_capture_setup(this->capture, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
  if (error != DC1394_SUCCESS) {
    LOG_ERROR("Failed to configure camera!");
    return -1;
  }

  // trigger mode
  // if (this->activateSoftwareTriggeringMode() != 0) {
  //   LOG_ERROR("Failed to activate software trigging!");
  //   return -1;
  // }
  if (this->activateDefaultTriggeringMode() != 0) {
    LOG_ERROR("Failed to activate defaul trigging!");
    return -1;
  }

  // video mode
  error = dc1394_video_set_mode(this->capture, DC1394_VIDEO_MODE_640x480_MONO8);
  if (error != DC1394_SUCCESS) {
    LOG_ERROR("Failed to set video mode!");
    return -1;
  }

  // iso speed
  error = dc1394_video_set_iso_speed(this->capture, DC1394_ISO_SPEED_400);
  if (error != DC1394_SUCCESS) {
    LOG_ERROR("Failed to set iso speed!");
    return -1;
  }

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
  if (error != DC1394_SUCCESS) {
    LOG_ERROR("Failed to start camera transmission!");
    return -1;
  }

  // update
  LOG_INFO("Camera initialized!");
  this->initialized = true;

  return 0;
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

int DC1394Camera::setTriggerMode(const int trigger_mode) {
  dc1394error_t err;

  // convert trigger mode from int to dc1394trigger_mode_t
  dc1394trigger_mode_t mode_value;
  if (trigger_mode == 0) {
    mode_value = DC1394_TRIGGER_MODE_0;
  } else if (trigger_mode == 1) {
    mode_value = DC1394_TRIGGER_MODE_1;
  } else if (trigger_mode == 2) {
    mode_value = DC1394_TRIGGER_MODE_2;
  } else if (trigger_mode == 3) {
    mode_value = DC1394_TRIGGER_MODE_3;
  } else if (trigger_mode == 4) {
    mode_value = DC1394_TRIGGER_MODE_4;
  } else if (trigger_mode == 5) {
    mode_value = DC1394_TRIGGER_MODE_5;
  } else if (trigger_mode == 14) {
    mode_value = DC1394_TRIGGER_MODE_14;
  } else if (trigger_mode == 15) {
    mode_value = DC1394_TRIGGER_MODE_15;
  } else {
    LOG_ERROR("Invalid trigger mode [%d]!", trigger_mode);
    return -1;
  }

  // set trigger mode
  err = dc1394_external_trigger_set_mode(this->capture, mode_value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not set trigger mode!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setTriggerSource(const int trigger_source) {
  dc1394trigger_source_t source_value;

  // convert trigger source from int to dc1394trigger_source_t
  if (trigger_source == 0) {
    source_value = DC1394_TRIGGER_SOURCE_0;
  } else if (trigger_source == 1) {
    source_value = DC1394_TRIGGER_SOURCE_1;
  } else if (trigger_source == 2) {
    source_value = DC1394_TRIGGER_SOURCE_2;
  } else if (trigger_source == 3) {
    source_value = DC1394_TRIGGER_SOURCE_3;
  } else if (trigger_source == 4) {
    source_value = DC1394_TRIGGER_SOURCE_SOFTWARE;
  }

  // set trigger source
  dc1394error_t err;
  err = dc1394_external_trigger_set_source(this->capture, source_value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not set trigger source!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setExternalTriggering(const bool activate) {
  // set external trigger
  const dc1394switch_t state = (activate) ? DC1394_ON : DC1394_OFF;

  dc1394error_t err;
  err = dc1394_external_trigger_set_power(this->capture, state);
  if (err != DC1394_SUCCESS && activate) {
    LOG_ERROR("Failed to activate external triggering!");
    return -1;
  } else if (err != DC1394_SUCCESS) {
    LOG_ERROR("Failed to de-activate external triggering!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setBrightness(const double brightness) {
  dc1394error_t err;

  // turn on the feature - dont know what this means??
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on the brightness feature!");
    return -1;
  }

  // turn off auto exposure
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_BRIGHTNESS,
                                DC1394_FEATURE_MODE_MANUAL);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn off Auto-brightness!");
    return -1;
  }

  // set brightness
  err = dc1394_feature_set_value(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 brightness);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not set value!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setFrameRate(const double fps) {
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
    LOG_ERROR("Unsupported or invalid frame rate %f!", fps);
    return -1;
  }

  // set camera frame rate
  dc1394error_t err = dc1394_video_set_framerate(this->capture, framerate);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Failed to set frame rate!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setExposure(const double exposure) {
  dc1394error_t err;

  // turn on the exposure feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_EXPOSURE,
                                 DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on the exposure feature!");
    return -1;
  }

  // turn off auto exposure
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_EXPOSURE,
                                DC1394_FEATURE_MODE_MANUAL);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn off Auto-exposure!");
    return -1;
  }

  // set exposure
  err = dc1394_feature_set_value(this->capture,
                                 DC1394_FEATURE_EXPOSURE,
                                 exposure);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not set exposure!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setShutter(const double shutter_ms) {
  dc1394error_t err;

  // turn on the shutter feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_SHUTTER,
                                 DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on the shutter feature!");
    return -1;
  }

  // turn off auto shutter
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_SHUTTER,
                                DC1394_FEATURE_MODE_MANUAL);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn off Auto-shutter!");
    return -1;
  }

  // set shutter
  err = dc1394_feature_set_value(this->capture,
                                 DC1394_FEATURE_SHUTTER,
                                 shutter_ms);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not set shutter!");
    return -1;
  }

  return 0;
}

int DC1394Camera::setGain(const double gain_db) {
  dc1394error_t err;

  // turn on the gain feature
  err = dc1394_feature_set_power(this->capture, DC1394_FEATURE_GAIN, DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on the gain feature!");
    return -1;
  }

  // turn off auto gain
  err = dc1394_feature_set_mode(this->capture,
                                DC1394_FEATURE_GAIN,
                                DC1394_FEATURE_MODE_MANUAL);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn off Auto-gain!");
    return -1;
  }

  // set gain
  err = dc1394_feature_set_value(this->capture, DC1394_FEATURE_GAIN, gain_db);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not set gain!");
    return -1;
  }

  return 0;
}

int DC1394Camera::getTriggerMode(int &trigger_mode) {
  dc1394error_t err;

  // get trigger mode
  dc1394trigger_mode_t mode_value;
  err = dc1394_external_trigger_get_mode(this->capture, &mode_value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get trigger mode!");
    return -1;
  }

  if (mode_value == DC1394_TRIGGER_MODE_0) {
    trigger_mode = 0;
  } else if (mode_value == DC1394_TRIGGER_MODE_1) {
    trigger_mode = 1;
  } else if (mode_value == DC1394_TRIGGER_MODE_2) {
    trigger_mode = 2;
  } else if (mode_value == DC1394_TRIGGER_MODE_3) {
    trigger_mode = 3;
  } else if (mode_value == DC1394_TRIGGER_MODE_4) {
    trigger_mode = 4;
  } else if (mode_value == DC1394_TRIGGER_MODE_5) {
    trigger_mode = 5;
  } else if (mode_value == DC1394_TRIGGER_MODE_14) {
    trigger_mode = 14;
  } else if (mode_value == DC1394_TRIGGER_MODE_15) {
    trigger_mode = 15;
  }

  return 0;
}

int DC1394Camera::getTriggerSource(int &trigger_source) {
  // get trigger source
  dc1394error_t err;
  dc1394trigger_source_t source_value;
  err = dc1394_external_trigger_get_source(this->capture, &source_value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get trigger source!");
    return -1;
  }

  // convert trigger source from dc1394trigger_source_t to int
  if (source_value == DC1394_TRIGGER_SOURCE_0) {
    trigger_source = 0;
  } else if (source_value == DC1394_TRIGGER_SOURCE_1) {
    trigger_source = 1;
  } else if (source_value == DC1394_TRIGGER_SOURCE_2) {
    trigger_source = 2;
  } else if (source_value == DC1394_TRIGGER_SOURCE_3) {
    trigger_source = 3;
  } else if (source_value == DC1394_TRIGGER_SOURCE_SOFTWARE) {
    trigger_source = 4;
  }

  return 0;
}

int DC1394Camera::getExternalTriggering(bool &activate) {
  // get external triggering state
  dc1394switch_t state;
  dc1394error_t err;

  err = dc1394_external_trigger_get_power(this->capture, &state);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get external trigger state!");
    return -1;
  }

  // convert external triggering state to bool
  activate = (state == DC1394_ON) ? true : false;

  return 0;
}

int DC1394Camera::getBrightness(double &brightness) {
  dc1394error_t err;

  // turn on the brightness feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on brightness feature!");
    return -1;
  }

  // get brightness value
  uint32_t value;
  err = dc1394_feature_get_value(this->capture,
                                 DC1394_FEATURE_BRIGHTNESS,
                                 &value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get brightness!");
    return -1;
  }
  brightness = (double) value;

  return 0;
}

int DC1394Camera::getFrameRate(double &fps) {
  dc1394framerate_t framerate;

  // get camera frame rate
  dc1394error_t err = dc1394_video_get_framerate(this->capture, &framerate);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Failed to get frame rate!");
    return -1;
  }

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
  dc1394error_t err;

  // turn on the exposure feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_EXPOSURE,
                                 DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on exposure feature!");
    return -1;
  }

  // get exposure value
  uint32_t value;
  err =
      dc1394_feature_get_value(this->capture, DC1394_FEATURE_EXPOSURE, &value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get exposure!");
    return -1;
  }
  exposure = (double) value;

  return 0;
}

int DC1394Camera::getShutter(double &shutter_ms) {
  dc1394error_t err;

  // turn on the exposure feature
  err = dc1394_feature_set_power(this->capture,
                                 DC1394_FEATURE_SHUTTER,
                                 DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on shutter feature!");
    return -1;
  }

  // get shutter value
  uint32_t value;
  err = dc1394_feature_get_value(this->capture, DC1394_FEATURE_SHUTTER, &value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get shutter!");
    return -1;
  }
  shutter_ms = (double) value;

  return 0;
}

int DC1394Camera::getGain(double &gain_db) {
  dc1394error_t err;

  // turn on the gain feature
  err = dc1394_feature_set_power(this->capture, DC1394_FEATURE_GAIN, DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not turn on gain feature!");
  }

  // get shutter value
  uint32_t value;
  err = dc1394_feature_get_value(this->capture, DC1394_FEATURE_GAIN, &value);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Could not get gain!");
  }
  gain_db = (double) value;

  return 0;
}

int DC1394Camera::getFrame(cv::Mat &image) {
  dc1394error_t err;

  // before calling dc1394_capture_dequeue(), wait for data to be available
  // using poll() since the former does not block. This allows a timeout so the
  // process does not hang forever if the capture is not ready (e.g. in
  // external trigger mode).
  //
  // tldr: block until ready to capture, timeout, or error
  pollfd poll_info;
  poll_info.fd = dc1394_capture_get_fileno(this->capture);
  poll_info.events = POLLIN;
  const int timeout_milliseconds = 10000;
  const int poll_result = poll(&poll_info, 1, timeout_milliseconds);

  // capture frame
  dc1394video_frame_t *frame = nullptr;
  if (poll_result > 0 && poll_info.revents == POLLIN) {
    err = dc1394_capture_dequeue(this->capture,
                                 DC1394_CAPTURE_POLICY_WAIT,
                                 &frame);
    if (err != DC1394_SUCCESS) {
      LOG_ERROR("Failed to obtain frame from camera!");
      return -1;
    }

  } else if (poll_result == 0) {
    LOG_ERROR("Camera guid:[%016lx] poll timed out", this->capture->guid);
    return -1;

  } else if (poll_result < 0) {
    LOG_ERROR("Camera guid:[%016lx] poll error: %s",
              this->capture->guid,
              strerror(errno));
    return -1;

  } else {
    LOG_ERROR("Camera guid:[%016lx] poll reported error: %d",
              this->capture->guid,
              poll_info.revents);
    return -1;
  }

  // convert mono 8 to colour
  bool debayer = false;
  const size_t channels = (debayer) ? 3 : 1;
  const size_t img_size = frame->image_bytes * channels;
  const size_t img_rows = frame->size[1];
  const size_t img_cols = frame->size[0];
  const size_t row_bytes = img_size / img_rows;

  // allocate memory for buffer
  if (this->buffer == nullptr) {
    this->buffer = (uint8_t *) malloc(img_size);
  }

  // decode bayer image
  if (debayer) {
    dc1394_bayer_decoding_8bit(frame->image,
                               this->buffer,
                               img_cols,
                               img_rows,
                               DC1394_COLOR_FILTER_BGGR,
                               DC1394_BAYER_METHOD_SIMPLE);
    // convert to opencv mat
    cv::Mat(img_rows, img_cols, CV_8UC3, this->buffer, row_bytes).copyTo(image);

  } else {
    // convert to opencv mat
    cv::Mat(img_rows, img_cols, CV_8UC1, frame->image, row_bytes).copyTo(image);
  }

  // release frame
  err = dc1394_capture_enqueue(this->capture, frame);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Failed to release frame buffer!");
    return -1;
  }

  return 0;
}

int DC1394Camera::activateSoftwareTriggeringMode() {
  int retval = 0;

  // trigger mode
  retval = this->setTriggerMode(0);
  if (retval != 0) {
    return -1;
  }

  // trigger source
  retval = this->setTriggerSource(4);
  if (retval != 0) {
    return -1;
  }

  // activate external triggering
  retval = this->setExternalTriggering(true);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int DC1394Camera::activateDefaultTriggeringMode() {
  int retval = 0;

  // trigger mode
  retval = this->setTriggerMode(3);
  if (retval != 0) {
    return -1;
  }

  // trigger source
  retval = this->setTriggerSource(0);
  if (retval != 0) {
    return -1;
  }

  // activate external triggering
  retval = this->setExternalTriggering(false);
  if (retval != 0) {
    return -1;
  }

  return 0;
}

int DC1394Camera::trigger() {
  const dc1394error_t err =
      dc1394_external_trigger_set_power(this->capture, DC1394_ON);
  if (err != DC1394_SUCCESS) {
    LOG_ERROR("Failed to software trigger camera!");
    return -1;
  }

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
