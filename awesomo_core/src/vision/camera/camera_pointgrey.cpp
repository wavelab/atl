#include "awesomo_core/vision/camera/camera_pointgrey.hpp"


namespace awesomo {

PointGreyCamera::PointGreyCamera(void) {
  this->pointgrey = NULL;
}

~PointGreyCamera(void) {
  if (initialized) {
    delete this->pointgrey;
  }
}

int PointGreyCamera::initialize() {
  int camera_index;
  FlyCapture2::Error error;
  FlyCapture2::Property property;
  FlyCapture2::Property gain_property;

  // setup
  this->pointgrey = new FlyCapture2::Camera();

  // connect
  camera_index = this->current_config->index;
  error = this->pointgrey->Connect(camera_index);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to connect to camera!");
    return -1;
  } else {
    log_err("Firefly camera connected!");
  }

  // set video mode format and frame rate
  error = this->pointgrey->SetVideoModeAndFrameRate(
    FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_60);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to set camera video mode and frame rate!");
    return -1;
  } else {
    log_err("Firefly camera video mode and frame rate configured!");
  }

  // configure exposure
  property.type = FlyCapture2::AUTO_EXPOSURE;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = this->camera_exposure_value;  // exposure value

  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to configure camera exposure!");
    return -1;
  } else {
    log_err("Firefly camera exposure configured!");
  }

  // configure gain
  property.type = FlyCapture2::GAIN;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = this->camera_gain_value;  // set the gain

  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to configure camera gain!");
    return -1;
  } else {
    log_err("Firefly camera gain configured!");
  }

  // start camera
  error = this->pointgrey->StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed start camera!");
    return -1;
  } else {
    log_err("Firefly initialized!");
  }
  this->initialized = true;

  return 0;
}

int Camera::getFrame(cv::Mat &image) {
  double data_size;
  double data_rows;
  unsigned int row_bytes;
  cv::Size image_size;

  FlyCapture2::Image raw_img;
  FlyCapture2::Image rgb_img;
  FlyCapture2::Error error;

  // obtain raw image
  error = this->pointgrey->RetrieveBuffer(&raw_img);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("Failed to obtain raw image from camera!");
    return -1;
  }

  // convert to rgb
  raw_img.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgb_img);

  // convert to opencv mat
  // clang-format off
  data_size = rgb_img.GetReceivedDataSize();
  data_rows = rgb_img.GetRows();
  row_bytes = data_size / data_rows;
  cv::Mat(rgb_img.GetRows(),
          rgb_img.GetCols(),
          CV_8UC3,
          rgb_img.GetData(),
          row_bytes).copyTo(image);
  // clang-format on

  // resize the image to reflect camera mode
  // clang-format off
  image_size = cv::Size(this->current_config->image_width,
                        this->current_config->image_height);
  cv::resize(image, image, image_size);
  // clang-format on

  return 0;
}

}  // end of awesomo namespace
