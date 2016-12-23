#include "awesomo_core/vision/camera_firefly.hpp"


namespace awesomo {

// MACROS
#define XIMEA_CHECK(STATE, WHERE)                \
  if (STATE != XI_OK) {                          \
    printf("Error after %s (%d)", WHERE, STATE); \
    goto ximea_error;                            \
  }


int XimeaCamera::configure(void) {
  int time_us;
  int ds_type;
  int ds_rate;
  float gain_db;
  int img_format;
  XI_RETURN retval;

  // setup
  time_us = this->camera_exposure_value;
  ds_type = 0;
  ds_rate = 2;
  gain_db = this->camera_gain_value;
  img_format = XI_RGB24;

  this->ximea = NULL;
  retval = XI_OK;

  log_info("Starting Ximea Camera\n");

  // open camera device
  retval = xiOpenDevice(0, &this->ximea);
  XIMEA_CHECK(retval, "xiOpenDevice");

  // exposure time
  retval = xiSetParamInt(this->ximea, XI_PRM_EXPOSURE, time_us);
  XIMEA_CHECK(retval, "xiSetParam (exposure time set)");

  // downsampling type
  retval = xiSetParamInt(this->ximea, XI_PRM_DOWNSAMPLING_TYPE, ds_type);
  XIMEA_CHECK(retval, "xiSetParam (downsampling type)");

  // downsampling
  retval = xiSetParamInt(this->ximea, XI_PRM_DOWNSAMPLING, ds_rate);
  XIMEA_CHECK(retval, "xiSetParam (downsampling rate)");

  // exposure gain
  retval = xiSetParamFloat(this->ximea, XI_PRM_GAIN, gain_db);
  XIMEA_CHECK(retval, "xiSetParam (gain)");

  // image format
  retval = xiSetParamInt(this->ximea, XI_PRM_IMAGE_DATA_FORMAT, img_format);
  XIMEA_CHECK(retval, "xiSetParam (image format)");

  // buffer policy
  // retval = xiSetParamInt(this->ximea, XI_PRM_BUFFER_POLICY, XI_BP_SAFE);
  // XIMEA_CHECK(retval, "xiSetParam (buffer policy)");

  // start acquisition
  retval = xiStartAcquisition(this->ximea);
  XIMEA_CHECK(retval, "xiStartAcquisition");

  // return
  log_info("Ximea Camera Started Successfully\n");
  return 0;

ximea_error:
  if (this->ximea) {
    xiCloseDevice(this->ximea);
  }
  return -1;
}

int XimeaCamera::getFrameXimea(cv::Mat &image) {
  int retval;
  XI_IMG ximea_img;

  // setup
  memset(&ximea_img, 0, sizeof(ximea_img));
  ximea_img.size = sizeof(XI_IMG);

  // get the image
  retval = xiGetImage(this->ximea, 1000, &ximea_img);
  if (retval != XI_OK) {
    log_error("Error after xiGetImage (%d)\n", retval);
    return -1;

  } else {
    // when ximea frame is mono
    // cv::Mat(frame.height, frame.width, CV_8U, frame.bp);

    // when ximea frame is rgb color (XI_RGB24 ONLY)
    cv::Mat(ximea_img.height, ximea_img.width, CV_8UC3, ximea_img.bp)
      .copyTo(image);

    // resize the image to reflect camera mode
    cv::resize(
      image,
      image,
      cv::Size(this->config->image_width, this->config->image_height));
    return 0;
  }
}

}  // end of awesomo namespace
