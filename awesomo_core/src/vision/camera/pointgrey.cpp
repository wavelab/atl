#include "awesomo_core/vision/camera/pointgrey.hpp"


namespace awesomo {

PointGreyCamera::PointGreyCamera(void) {
  this->pointgrey = NULL;
}

PointGreyCamera::~PointGreyCamera(void) {
  if (initialized) {
    delete this->pointgrey;
  }
}

int PointGreyCamera::initialize() {
  int camera_index;
  FlyCapture2::Error error;
  FlyCapture2::Property property;
  FlyCapture2::Property gain_property;

  FlyCapture2::PGRGuid guid;
  FlyCapture2::BusManager bus_manager;

  // setup
  this->pointgrey = new FlyCapture2::Camera();

  // look for camera
  error = bus_manager.GetCameraFromIndex(0, &guid);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Could not find a camera!");
    return -1;
  }

  error = this->pointgrey->Connect(&guid);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to connect to camera!");
    return -1;
  } else {
    log_info("PointGrey camera connected!");
  }

  // this->printFormat7Capabilities();
  // this->setFormat7("raw8", 640, 480, 0); // make these not hard coded

  // set video mode format and frame RRRR USB 2.0 cameras
  // error = this->pointgrey->SetVideoModeAndFrameRate(
  //   FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_60);
  // if (error != FlyCapture2::PGRERROR_OK) {
  //   log_err("ERROR! Failed to set camera video mode and frame rate!");
  //   return -1;
  // } else {
  //   log_err("PointGrey camera video mode and frame rate configured!");
  // }

  this->setExposure(this->config.exposure_value);
  this->setGain(this->config.gain_value);
  this->setFrameRate(200);
  this->setFormat7("RAW8", 640 * 2, 480 * 2, 0); // make these not hard coded

  // start camera
  error = this->pointgrey->StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed start camera!");
    return -1;
  } else {
    log_info("PointGrey initialized!");
  }
  this->initialized = true;

  return 0;
}

int PointGreyCamera::getFrame(cv::Mat &image) {
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
  image_size = cv::Size(this->config.image_width,
                        this->config.image_height);
  cv::resize(image, image, image_size);
  // clang-format on

  return 0;
}

int PointGreyCamera::setFormat7(std::string pixel_format,
                                int crop_width,
                                int crop_height,
                                int mode) {

  FlyCapture2::Format7ImageSettings fmt7_settings;
  FlyCapture2::PixelFormat k_fmt7PixFmt;
  FlyCapture2::Error error;
  FlyCapture2::Format7PacketInfo fmt7PacketInfo;
  bool valid;
  int max_image_width;
  int max_image_height;


  if (pixel_format == "MONO8") {
    fmt7_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
  } else if (pixel_format == "MONO16") {
    fmt7_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO16;
  } else if (pixel_format == "RAW8") {
    fmt7_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
  } else if (pixel_format == "RAW16") {
    fmt7_settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW16;
  }

  switch (mode) {
    case 0:
      fmt7_settings.mode = FlyCapture2::MODE_0;
      max_image_width = 2048;
      max_image_height = 1536;
      break;
    case 1:
      fmt7_settings.mode = FlyCapture2::MODE_1;
      max_image_width = 1024;
      max_image_height = 768;
      break;
    case 2:
      fmt7_settings.mode = FlyCapture2::MODE_2;
      max_image_width = 1024;
      max_image_height = 738;
      break;
  }

  fmt7_settings.offsetX = (max_image_width / 2 )  - crop_width / 2;
  fmt7_settings.offsetY = (max_image_height / 2 ) - crop_height / 2;
  fmt7_settings.width = crop_width;
  fmt7_settings.height = crop_height;

  // Validate the Format7 settings
  error = this->pointgrey->ValidateFormat7Settings(&fmt7_settings,
                                                   &valid,
                                                   &fmt7PacketInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("Format7 settings are invalid!, could not configure the camera");
    return -1;
  }

  // Send the settings to the camera with recommended packet size
  error = this->pointgrey->SetFormat7Configuration(&fmt7_settings,
      fmt7PacketInfo.recommendedBytesPerPacket);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("Could not configure the camera with Format7!");
    return -1;
  }
  log_info("Format7 Settings applied successfully!");

  return 0;
}

int PointGreyCamera::setFrameRate(double frame_rate) {
  FlyCapture2::Error error;
  FlyCapture2::Property fps_prop;

  fps_prop.type = FlyCapture2::FRAME_RATE;
  fps_prop.onOff = true;
  fps_prop.autoManualMode = false;
  fps_prop.absControl = true;
  fps_prop.absValue = frame_rate;

  error = this->pointgrey->SetProperty(&fps_prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to configure camera frame_rate!");
    return -1;
  } else {
    log_info("PointGrey camera frame_rate set to %3.2f", frame_rate);
  }

  return 0;
}

int PointGreyCamera::setExposure(double exposure) {
  FlyCapture2::Error error;
  FlyCapture2::Property exposure_prop;

  exposure_prop.type = FlyCapture2::AUTO_EXPOSURE;
  exposure_prop.onOff = true;
  exposure_prop.autoManualMode = false;
  exposure_prop.absControl = true;
  exposure_prop.absValue = exposure;

  error = this->pointgrey->SetProperty(&exposure_prop);

  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to configure camera exposure!");
    return -1;
  } else {
    log_info("PointGrey camera exposure set to %3.2f", exposure);
  }
  return 0;
}

int PointGreyCamera::setGain(double gain) {
  FlyCapture2::Error error;
  FlyCapture2::Property gain_prop;

  gain_prop.type = FlyCapture2::GAIN;
  gain_prop.onOff = true;
  gain_prop.autoManualMode = false;
  gain_prop.absControl = true;
  gain_prop.absValue = gain;

  error = this->pointgrey->SetProperty(&gain_prop);

  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("ERROR! Failed to configure camera gain!");
    return -1;
  } else {
    log_info("PointGrey camera gain set to %3.2f", gain);
  }
  return 0;
}

int PointGreyCamera::printFormat7Capabilities(void) {
  bool supported;
  FlyCapture2::Format7Info fmt7_info;

  this->pointgrey->GetFormat7Info(&fmt7_info, &supported);
  log_info(
      "Max image pixels: (%u, %u)\n"
      "Image Unit size: (%u, %u)\n"
      "Offset Unit size: (%u, %u)\n"
      "Pixel format bitfield: 0x%08x\n",
      fmt7_info.maxWidth, fmt7_info.maxHeight, fmt7_info.imageHStepSize,
      fmt7_info.imageVStepSize, fmt7_info.offsetHStepSize,
      fmt7_info.offsetVStepSize, fmt7_info.pixelFormatBitField
  );

  return 0;
}

}  // end of awesomo namespace
