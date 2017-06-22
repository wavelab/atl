#include "wavesim_gazebo/clients/camera_gclient.hpp"


namespace wavesim {
namespace gaz {

CameraGClient::CameraGClient(void) {
  this->connected = false;
}

CameraGClient::~CameraGClient(void) {
  if (this->connected) {
    gazebo::client::shutdown();
  }
}

int CameraGClient::configure(void) {
  // pre-check
  this->connected = gazebo::client::setup(0, NULL);
  if (this->connected == false) {
    return -1;
  }

  // setup gazebo node
  GazeboNode::configure();
  this->registerSubscriber(IMAGE_TOPIC, &CameraGClient::imageCallback, this);
  this->waitForConnection();
  return 0;
}

void CameraGClient::imageCallback(ConstImagePtr &msg) {
  int buf_size;
  unsigned char *img_data;

  // convert img message to cv::Mat
  buf_size = (int) msg->width() * (int) msg->height() * 3;
  img_data = new unsigned char[buf_size + 1];
  memcpy((char *) img_data, msg->data().c_str(), buf_size);

  // clang-format off
  this->image = cv::Mat(
    (int) msg->height(),
    (int) msg->width(),
    CV_8UC3,
    img_data
  ).clone();
  // clang-format on

  // debug
  // cv::imshow("CameraGClient Image", this->image);
  // cv::waitKey(1);

  // clean up
  delete img_data;
}

}  // end of gaz namespace
}  // end of wavesim namespace
