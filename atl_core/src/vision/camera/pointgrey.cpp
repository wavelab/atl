#include "atl/vision/camera/pointgrey.hpp"

namespace atl {

PointGreyCamera::~PointGreyCamera() {
  FlyCapture2::Error error;

  if (this->initialized && this->pointgrey) {
    // stop capture
    error = this->pointgrey->StopCapture();
    if (error != FlyCapture2::PGRERROR_OK) {
      LOG_ERROR("Failed to stop PointGreyCamera capture!");
    }

    // disconnect
    error = this->pointgrey->Disconnect();
    if (error != FlyCapture2::PGRERROR_OK) {
      LOG_ERROR("Failed to disconnect PointGreyCamera!");
    }

    // delete pointgrey
    delete this->pointgrey;
  }
}

int PointGreyCamera::initialize() {
  FlyCapture2::Error error;
  FlyCapture2::Property property;
  FlyCapture2::Property gain_property;

  FlyCapture2::BusManager bus_manager;

  // setup
  this->pointgrey = new FlyCapture2::Camera();

  // look for camera
  FlyCapture2::PGRGuid guid;
  error = bus_manager.GetCameraFromIndex(0, &guid);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    LOG_ERROR("ERROR! Could not find a camera!");
    return -1;
  }

  // display camera info
  FlyCapture2::CameraInfo cam_info;
  error = this->pointgrey->GetCameraInfo(&cam_info);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    LOG_ERROR("ERROR! Failed to get camera info from camera!");
    return -1;
  } else {
    LOG_INFO(
      "PointGrey [%s] - serial no. [%d]",
      cam_info.modelName,
      cam_info.serialNumber);
  }

  // connect
  error = this->pointgrey->Connect(&guid);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! Failed to connect to camera!");
    return -1;
  } else {
    LOG_INFO("PointGrey connected!");
  }

  // this->setFrameRate(200);
  // this->setExposure(this->config.exposure_value);
  // this->setGain(this->config.gain_value);
  // this->setShutter(this->config.shutter_speed);
  // this->setFormat7(1, "RAW8", 1024, 768);

  // start camera
  error = this->pointgrey->StartCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! Failed start camera!");
    return -1;
  } else {
    LOG_INFO("PointGrey initialized!");
  }

  this->initialized = true;
  return 0;
}

int PointGreyCamera::setBrightness(double brightness) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::BRIGHTNESS;
  property.absControl = true;
  property.absValue = brightness;

  // set frame rate
  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! Failed to set BRIGHTNESS property!");
    return -1;
  }

  return 0;
}

int PointGreyCamera::setFrameRate(double fps) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::FRAME_RATE;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = fps;

  // set frame rate
  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to set FRAME_RATE property!");
    return -1;
  }

  return 0;
}

int PointGreyCamera::setExposure(double exposure_val) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::AUTO_EXPOSURE;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = exposure_val;

  // set exposure
  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to set EXPOSURE property!");
    return -1;
  }

  return 0;
}

int PointGreyCamera::setShutter(double shutter_ms) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::SHUTTER;
  property.onOff = true;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = shutter_ms;

  // set shutter
  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to set SHUTTER property!");
    return -1;
  }

  return 0;
}

int PointGreyCamera::setGain(double gain_db) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::GAIN;
  property.autoManualMode = false;
  property.absControl = true;
  property.absValue = gain_db;

  // set shutter
  error = this->pointgrey->SetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to set GAIN property!");
    return -1;
  }

  return 0;
}

int PointGreyCamera::getBrightness(double &brightness) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::BRIGHTNESS;

  // get frame rate
  error = this->pointgrey->GetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to set BRIGHTNESS property!");
    return -1;
  }
  brightness = property.absValue;

  return 0;
}

int PointGreyCamera::getFrameRate(double &fps) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::FRAME_RATE;

  // get frame rate
  error = this->pointgrey->GetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to set FRAME_RATE property!");
    return -1;
  }
  fps = property.absValue;

  return 0;
}

int PointGreyCamera::getExposure(double &exposure_val) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::AUTO_EXPOSURE;

  // set exposure
  error = this->pointgrey->GetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to get EXPOSURE property!");
    return -1;
  }
  exposure_val = property.absValue;

  return 0;
}

int PointGreyCamera::getShutter(double &shutter_ms) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::SHUTTER;

  // get shutter
  error = this->pointgrey->GetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to get SHUTTER property!");
    return -1;
  }
  shutter_ms = property.absValue;

  return 0;
}

int PointGreyCamera::getGain(double &gain_db) {
  FlyCapture2::Property property;
  FlyCapture2::Error error;

  // build property struct
  property.type = FlyCapture2::GAIN;

  // get shutter
  error = this->pointgrey->GetProperty(&property);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("ERROR! failed to get GAIN property!");
    return -1;
  }
  gain_db = property.absValue;

  return 0;
}

int PointGreyCamera::printFormat7Capabilities() {
  bool supported;
  FlyCapture2::Format7Info info;

  // pre-check
  if (this->initialized == false) {
    return -1;
  }

  // get format7 info
  this->pointgrey->GetFormat7Info(&info, &supported);
  LOG_INFO(
    "Max image pixels: (%u, %u)\n"
    "Image Unit size: (%u, %u)\n"
    "Offset Unit size: (%u, %u)\n"
    "Pixel format bitfield: 0x%08x\n",
    info.maxWidth,
    info.maxHeight,
    info.imageHStepSize,
    info.imageVStepSize,
    info.offsetHStepSize,
    info.offsetVStepSize,
    info.pixelFormatBitField);

  return 0;
}

int PointGreyCamera::setFormat7(
  int mode, std::string pixel_format, int width, int height) {
  bool valid;
  unsigned int packet_size;
  float psize_percentage;
  FlyCapture2::Format7Info info;
  FlyCapture2::Error error;
  FlyCapture2::Format7PacketInfo packet_info;
  FlyCapture2::Format7ImageSettings settings;

  // get format7 settings
  this->pointgrey->GetFormat7Configuration(
    &settings, &packet_size, &psize_percentage);

  // set mode
  switch (mode) {
    case 0: settings.mode = FlyCapture2::MODE_1; break;
    case 1: settings.mode = FlyCapture2::MODE_1; break;
    case 2: settings.mode = FlyCapture2::MODE_2; break;
    default:
      LOG_ERROR("Format7 mode [%d] not implemented yet!", mode);
      return -2;
  }
  settings.width = width;
  settings.height = height;
  settings.offsetX = 0;
  settings.offsetY = 0;
  if (pixel_format == "MONO8") {
    settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
  } else if (pixel_format == "MONO16") {
    settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO16;
  } else if (pixel_format == "RAW8") {
    settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
  } else if (pixel_format == "RAW16") {
    settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW16;
  } else {
    LOG_ERROR("PixelFormat [%s] not implemented yet!", pixel_format.c_str());
    return -2;
  }

  // validate settings
  error =
    this->pointgrey->ValidateFormat7Settings(&settings, &valid, &packet_info);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR(
      "Format7 settings are invalid!, could not configure the camera");
    return -1;
  }

  // send settings
  error = this->pointgrey->SetFormat7Configuration(
    &settings, packet_info.maxBytesPerPacket);
  if (error != FlyCapture2::PGRERROR_OK) {
    LOG_ERROR("Could not configure the camera with Format7!");
    return -1;
  }
  LOG_INFO("Format7 Settings applied successfully!");

  return 0;
}

std::pair<int, int> PointGreyCamera::centerROI(
  int size, int max_size, int step) {
  if (size == 0 || size > max_size) {
    size = max_size;
  }

  // size must be a multiple of the step
  size = size / step * step;
  const int offset = (max_size - size) / 2;
  return std::make_pair(size, offset);
}

int PointGreyCamera::changeMode(std::string mode) {
  // pre-check
  if (this->configs.find(mode) == this->configs.end()) {
    return -1;
  }

  // update camera settings
  this->config = this->configs[mode];

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
    LOG_ERROR("Failed to obtain raw image from camera!");
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
  image_size = cv::Size(this->config.image_width, this->config.image_height);
  cv::resize(image, image, image_size, 0, 0, cv::INTER_NEAREST);

  return 0;
}

int PointGreyCamera::run() {
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

}  // namespace atl
