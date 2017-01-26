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

  this->printFormat7Capabilities();
  this->setFormat7();

  // set video mode format and frame rate
  // error = this->pointgrey->SetVideoModeAndFrameRate(
  //   FlyCapture2::VIDEOMODE_640x480Y8, FlyCapture2::FRAMERATE_60);
  // if (error != FlyCapture2::PGRERROR_OK) {
  //   log_err("ERROR! Failed to set camera video mode and frame rate!");
  //   return -1;
  // } else {
  //   log_err("PointGrey camera video mode and frame rate configured!");
  // }
  //
  // // configure exposure
  // property.type = FlyCapture2::AUTO_EXPOSURE;
  // property.onOff = true;
  // property.autoManualMode = false;
  // property.absControl = true;
  // // property.absValue = this->camera_exposure_value;  // exposure value
  //
  // error = this->pointgrey->SetProperty(&property);
  // if (error != FlyCapture2::PGRERROR_OK) {
  //   log_err("ERROR! Failed to configure camera exposure!");
  //   return -1;
  // } else {
  //   log_err("PointGrey camera exposure configured!");
  // }
  //
  // // configure gain
  // property.type = FlyCapture2::GAIN;
  // property.onOff = true;
  // property.autoManualMode = false;
  // property.absControl = true;
  // // property.absValue = this->camera_gain_value;  // set the gain
  //
  // error = this->pointgrey->SetProperty(&property);
  // if (error != FlyCapture2::PGRERROR_OK) {
  //   log_err("ERROR! Failed to configure camera gain!");
  //   return -1;
  // } else {
  //   printf("PointGrey camera gain configured!");
  // }
  //
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
  // image_size = cv::Size(this->current_config->image_width,
  //                       this->current_config->image_height);
  // image_size = cv::Size(480, 320);
  // cv::resize(image, image, image_size);
  // clang-format on

  return 0;
}

int PointGreyCamera::setFormat7(void) {

  FlyCapture2::Format7ImageSettings fmt7_image_settings;
  FlyCapture2::PixelFormat k_fmt7PixFmt = FlyCapture2::PIXEL_FORMAT_BGR;
  FlyCapture2::Error error;
  FlyCapture2::Format7PacketInfo fmt7PacketInfo;
  bool valid;

  k_fmt7PixFmt = FlyCapture2::PIXEL_FORMAT_RAW8;

  fmt7_image_settings.mode = FlyCapture2::MODE_0;
  fmt7_image_settings.offsetX = 380;
  fmt7_image_settings.offsetY = 298;
  fmt7_image_settings.width = 1264;
  fmt7_image_settings.height = 930;
  fmt7_image_settings.pixelFormat = k_fmt7PixFmt;


  //Validate the settings to make sure that they are valid
  error =
    this->pointgrey->ValidateFormat7Settings(&fmt7_image_settings, &valid, &fmt7PacketInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("Format7 settings are invalid!");
    return -1;
  }
  // Send the settings to the camera
  error = this->pointgrey->SetFormat7Configuration(&fmt7_image_settings,
      fmt7PacketInfo.recommendedBytesPerPacket);

  if (error != FlyCapture2::PGRERROR_OK) {
    log_err("Could not configure the camera with Format7!");
    return -1;
  }

    log_info("Format7 Settings applied successfully!");
    this->setFrameRate(200);
    return 0;
}

int PointGreyCamera::setFrameRate(double fps)
{
  FlyCapture2::PropertyInfo prop_info;
  prop_info.type = FlyCapture2::FRAME_RATE;
  this->pointgrey->GetPropertyInfo(&prop_info);
  FlyCapture2::Property fps_prop;

  fps_prop.type = FlyCapture2::FRAME_RATE;
  fps_prop.onOff = true && prop_info.autoSupported;
  fps_prop.autoManualMode = false && prop_info.autoSupported;
  fps_prop.absControl = prop_info.absValSupported;
  fps_prop.absValue = fps;
  this->pointgrey->SetProperty(&fps_prop);
  return 0;

}

int PointGreyCamera::printFormat7Capabilities(void) {
  FlyCapture2::Format7Info fmt7_info;
  bool supported;

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
