#ifndef ATL_CORE_VISION_CAMERA_DC1394_HPP
#define ATL_CORE_VISION_CAMERA_DC1394_HPP

#include <inttypes.h>
#include <stdio.h>

#include <dc1394/dc1394.h>

#include "atl/utils/utils.hpp"
#include "atl/vision/camera/camera.hpp"

namespace atl {

class DC1394Camera : public Camera {
public:
  dc1394_t *dc1394 = nullptr;
  dc1394camera_t *capture = nullptr;
  uint8_t *buffer = nullptr;

  DC1394Camera() {}
  ~DC1394Camera() {
    // free buffer
    if (this->buffer != nullptr) {
      free(buffer);
    }

    // close capture
    if (this->capture != nullptr) {
      dc1394_video_set_transmission(this->capture, DC1394_OFF);
      dc1394_capture_stop(this->capture);
      dc1394_camera_free(this->capture);
    }

    // close connection to libdc1394
    if (this->dc1394 != nullptr) {
      dc1394_free(this->dc1394);
      this->dc1394 = nullptr;
    }
  }

  /**
   * Initialize camera
   *
   * @param guid Global Unique ID (default = 0; initialize first available
   * camera)
   * @return 0 for success, -1 for failure
   */
  int initialize(uint64_t guid = 0);

  /**
   * Print frame information
   * @param frame Camera frame data
   */
  void printFrameInfo(dc1394video_frame_t *frame);

  /**
   * Set brightness
   *
   * @param brightness
   * @return 0 for success, -1 for failure
   */
  int setBrightness(double brightness);

  /**
   * Set frame rate
   *
   * @param frame rate
   * @return 0 for success, -1 for failure
   */
  int setFrameRate(double fps);

  /**
   * Set exposure
   *
   * @param exposure
   * @return 0 for success, -1 for failure
   */
  int setExposure(double exposure);

  /**
   * Set shutter
   *
   * @param shutter
   * @return 0 for success, -1 for failure
   */
  int setShutter(double shutter_ms);

  /**
   * Set gain
   *
   * @param gain
   * @return 0 for success, -1 for failure
   */
  int setGain(double gain_db);

  /**
   * Get brightness
   *
   * @param brightness
   * @return 0 for success, -1 for failure
   */
  int getBrightness(double &brightness);

  /**
   * Get frame rate
   *
   * @param frame rate
   * @return 0 for success, -1 for failure
   */
  int getFrameRate(double &fps);

  /**
   * Get exposure
   *
   * @param exposure
   * @return 0 for success, -1 for failure
   */
  int getExposure(double &exposure);

  /**
   * Get shutter speed
   *
   * @param shutter speed
   * @return 0 for success, -1 for failure
   */
  int getShutter(double &shutter_ms);

  /**
   * Get gain
   *
   * @param gain
   * @return 0 for success, -1 for failure
   */
  int getGain(double &gain_db);

  /**
   * Calculate center ROI
   *
   * @param size
   * @param max_size
   * @param step
   *
   * @return Center of ROI
   */
  std::pair<int, int> centerROI(const int size,
                                const int max_size,
                                const int step);

  /**
   * Change mode
   *
   * @param mode Mode
   * @return 0 for success, -1 for failure
   */
  int changeMode(const std::string &mode);

  /**
   * Get frame
   *
   * @param image Image
   * @return 0 for success, -1 for failure
   */
  int getFrame(cv::Mat &image);

  /**
   * Run
   *
   * @return 0 for success, -1 for failure
   */
  int run();
};

} // namespace atl
#endif
