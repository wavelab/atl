#ifndef ATL_CORE_VISION_CAMERA_DC1394_HPP
#define ATL_CORE_VISION_CAMERA_DC1394_HPP

#include <inttypes.h>
#include <poll.h>
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
   * Connect to camera
   *
   * @param guid Global Unique ID (default = 0; initialize first available
   * camera)
   * @return 0 for success, -1 for failure
   */
  int connect(uint64_t guid = 0);

  /**
   * Initialize camera
   *
   * @param guid Global Unique ID (default = 0; initialize first available
   * camera)
   * @return 0 for success, -1 for failure
   */
  int initialize(uint64_t guid = 0);

  /**
   * Change mode
   *
   * @param mode Mode
   * @return 0 for success, -1 for failure
   */
  int changeMode(const std::string &mode);

  /**
   * Print frame information
   * @param frame Camera frame data
   */
  void printFrameInfo(dc1394video_frame_t *frame);

  /**
   * Set camera trigger mode
   *
   * Mode 0: Exposure starts with a falling edge and stops when the the
   * exposure specified by the SHUTTER feature is elapsed.
   *
   * Mode 1: Exposure starts with a falling edge and stops with the next rising
   * edge.
   *
   * Mode 2: The camera starts the exposure at the first falling edge and stops
   * the integration at the nth falling edge. The parameter n is a prameter of
   * the trigger that can be set with dc1394_feature_set_value().
   *
   * Mode 3: This is an internal trigger mode. The trigger is generated every
   * n * (period of fastest framerate). Once again, the parameter n can be set
   * with dc1394_feature_set_value().
   *
   * Mode 4: A multiple exposure mode. N exposures are performed each time a
   * falling edge is observed on the trigger signal.  Each exposure is as long
   * as defined by the SHUTTER feature.
   *
   * Mode 5: Another multiple exposure mode. Same as Mode 4 except that the
   * exposure is is defined by the length of the trigger pulse instead of the
   * SHUTTER feature.
   *
   * Mode 14 and 15: vendor specified trigger mode.
   *
   * @param trigger_mode Trigger mode
   * @return 0 for success, -1 for failure
   */
  int setTriggerMode(const int trigger_mode);

  /**
   * Set camera trigger source
   *
   * @param trigger_source Trigger source
   *    - 0: Trigger source 0
   *    - 1: Trigger source 1
   *    - 2: Trigger source 2
   *    - 3: Trigger source 3
   *    - 4: Trigger software trigger
   *
   * @return 0 for success, -1 for failure
   */
  int setTriggerSource(const int trigger_mode);

  /**
   * Set camera external triggering state
   *
   * @param activate Activate
   * @return 0 for success, -1 for failure
   */
  int setExternalTriggering(const bool activate);

  /**
   * Set brightness
   *
   * @param brightness
   * @return 0 for success, -1 for failure
   */
  int setBrightness(const double brightness);

  /**
   * Set frame rate
   *
   * @param frame rate
   * @return 0 for success, -1 for failure
   */
  int setFrameRate(const double fps);

  /**
   * Set exposure
   *
   * @param exposure
   * @return 0 for success, -1 for failure
   */
  int setExposure(const double exposure);

  /**
   * Set shutter
   *
   * @param shutter
   * @return 0 for success, -1 for failure
   */
  int setShutter(const double shutter_ms);

  /**
   * Set gain
   *
   * @param gain
   * @return 0 for success, -1 for failure
   */
  int setGain(const double gain_db);

  /**
   * Get camera trigger mode
   *
   * Mode 0: Exposure starts with a falling edge and stops when the the
   * exposure specified by the SHUTTER feature is elapsed.
   *
   * Mode 1: Exposure starts with a falling edge and stops with the next rising
   * edge.
   *
   * Mode 2: The camera starts the exposure at the first falling edge and stops
   * the integration at the nth falling edge. The parameter n is a prameter of
   * the trigger that can be set with dc1394_feature_set_value().
   *
   * Mode 3: This is an internal trigger mode. The trigger is generated every
   * n * (period of fastest framerate). Once again, the parameter n can be set
   * with dc1394_feature_set_value().
   *
   * Mode 4: A multiple exposure mode. N exposures are performed each time a
   * falling edge is observed on the trigger signal.  Each exposure is as long
   * as defined by the SHUTTER feature.
   *
   * Mode 5: Another multiple exposure mode. Same as Mode 4 except that the
   * exposure is is defined by the length of the trigger pulse instead of the
   * SHUTTER feature.
   *
   * Mode 14 and 15: vendor specified trigger mode.
   *
   * @param trigger_mode Trigger mode
   * @return 0 for success, -1 for failure
   */
  int getTriggerMode(int &trigger_mode);

  /**
   * Get camera trigger source
   *
   * @param trigger_source Trigger source
   *    - 0: Trigger source 0
   *    - 1: Trigger source 1
   *    - 2: Trigger source 2
   *    - 3: Trigger source 3
   *    - 4: Trigger software trigger
   *
   * @return 0 for success, -1 for failure
   */
  int getTriggerSource(int &trigger_source);

  /**
   * Get camera external triggering state
   *
   * @param activate Activate
   * @return 0 for success, -1 for failure
   */
  int getExternalTriggering(bool &activate);

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
   * Postprocess image
   *
   * @param image Image
   * @return 0 for success, -1 for failure
   */
  int postprocessImage(cv::Mat &image, const dc1394video_frame_t *frame);

  /**
   * Get frame
   *
   * @param image Image
   * @return 0 for success, -1 for failure
   */
  int getFrame(cv::Mat &image);

  /**
   * Activate software triggering mode
   *
   * @return 0 for success, -1 for failure
   */
  int activateSoftwareTriggeringMode();

  /**
   * Activate default triggering mode
   *
   * @return 0 for success, -1 for failure
   */
  int activateDefaultTriggeringMode();

  /**
   * Software trigger
   *
   * @return 0 for success, -1 for failure
   */
  int trigger();

  /**
   * Run
   *
   * @return 0 for success, -1 for failure
   */
  int run();
};

} // namespace atl
#endif
