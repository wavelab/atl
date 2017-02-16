#include "awesomo_core/vision/camera/ximea.hpp"


namespace awesomo {

XimeaCamera::XimeaCamera(void) {
  this->ximea= NULL;
}


int XimeaCamera::initialize(void) {
  int ds_type;
  int ds_rate;
  int img_format;
  XI_RETURN retval;
  int image_height;
  int image_width;
  int offset_x;
  int offset_y;


  // setup
  ds_type = 0;
  ds_rate = 1;
  // img_format = XI_RAW8;
  img_format = XI_MONO8;

  image_width = 1280;
  image_height = 480 * 2;
  // offset_y = 100;
  // offset_x = 100;
  offset_y = 0;
  offset_x = 0;

  this->ximea = NULL;
  retval = XI_OK;

  log_info("Starting Ximea Camera\n");

  // clang-format off
  // open camera device
  retval = xiOpenDevice(0, &this->ximea);
  this->checkState(retval, "xiOpenDevice");

  // downsampling type
  retval = xiSetParamInt(this->ximea, XI_PRM_DOWNSAMPLING_TYPE, ds_type);
  this->checkState(retval, "xiSetParam (downsampling type)");

  // downsampling
  retval = xiSetParamInt(this->ximea, XI_PRM_DOWNSAMPLING, ds_rate);
  this->checkState(retval, "xiSetParam (downsampling rate)");

  // image format
  retval = xiSetParamInt(this->ximea, XI_PRM_IMAGE_DATA_FORMAT, img_format);
  this->checkState(retval, "xiSetParam (image format)");

  // windowing
  retval = xiSetParamInt(this->ximea, XI_PRM_WIDTH, image_width);
  this->checkState(retval, "xiSetParam (image_width)");

  retval = xiSetParamInt(this->ximea, XI_PRM_HEIGHT, image_height);
  this->checkState(retval, "xiSetParam (image_height)");

  retval = xiSetParamInt(this->ximea, XI_PRM_OFFSET_X, offset_x);
  this->checkState(retval, "xiSetParam (offset_x)");

  retval = xiSetParamInt(this->ximea, XI_PRM_OFFSET_Y, offset_y);
  this->checkState(retval, "xiSetParam (offset_y)");

  // Gain and Exposure
  this->setExposure(this->config.exposure_value);
  this->setGain(this->config.gain_value);

  // buffer policy
  // retval = xiSetParamInt(this->ximea, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
  // checkState(retval, "xiSetParam (buffer policy)");

  // start acquisition
  retval = xiStartAcquisition(this->ximea);
  this->checkState(retval, "xiStartAcquisition");
  // clang-format on

  // return
  log_info("Ximea Camera Started Successfully\n");
  return 0;

// ximea_error:
//   if (this->ximea) {
//     xiCloseDevice(this->ximea);
//   }
//   return -1;
// }
}

int XimeaCamera::checkState(XI_RETURN retval, std::string where) {
  if (retval != XI_OK) {
    std::cout << "Error after " << where << std::endl;
    if (this->ximea) {
      xiCloseDevice(this->ximea);
      return -1;
    } else {
      return 0;
    }
  }
}


int XimeaCamera::setGain(float gain_db)
{
  XI_RETURN retval;

  retval = xiSetParamFloat(this->ximea, XI_PRM_GAIN, gain_db);
  return this->checkState(retval, "xiSetParam (exposure time set)");
}

int XimeaCamera::setExposure(float exposure_time_us) {
  XI_RETURN retval;

  retval = xiSetParamInt(this->ximea, XI_PRM_EXPOSURE, exposure_time_us);
  return this->checkState(retval, "xiSetParam (exposure time set)");
}

int XimeaCamera::changeMode(std::string mode) {
  // pre-check
  if (this->configs.find(mode) == this->configs.end()) {
    return -1;
  }
  // update camera settings
  this->config = this->configs[mode];
  return 0;
}


int XimeaCamera::getFrame(cv::Mat &image) {
  int retval;
  XI_IMG ximea_img;

  // setup
  memset(&ximea_img, 0, sizeof(ximea_img));
  ximea_img.size = sizeof(XI_IMG);

  // get the image
  retval = xiGetImage(this->ximea, 1000, &ximea_img);
  if (retval != XI_OK) {
    log_err("Error after xiGetImage (%d)\n", retval);
    return -1;

  } else {
    // when ximea frame is mono
    // cv::Mat(frame.height, frame.width, CV_8U, frame.bp);

    // when ximea frame is rgb color (XI_RGB24 ONLY)
    // cv::Mat(ximea_img.height, ximea_img.width, CV_8UC3, ximea_img.bp)
    //   .copyTo(image);
    cv::Mat(ximea_img.height, ximea_img.width, CV_8U, ximea_img.bp)
      .copyTo(image);

    // resize the image to reflect camera mode
    cv::resize(
      image,
      image,
      cv::Size(640, 480));
    // cv::Size(this->config.image_width, this->config.image_height));
    return 0;
  }
}

}  // end of awesomo namespace
