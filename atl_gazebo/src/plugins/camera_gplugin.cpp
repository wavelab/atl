#include "atl/gazebo/plugins/camera_gplugin.hpp"

namespace atl {
namespace gaz {

CameraGPlugin::CameraGPlugin() { printf("LOADING [libcamera_gplugin.so]!\n"); }

void CameraGPlugin::Load(gazebo::sensors::SensorPtr sptr, sdf::ElementPtr sdf) {
  UNUSED(sdf);

  // load sensor pointer
  // clang-format off
  this->sensor = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sptr);
  if (!this->sensor) {
    gzerr << "CameraGPlugin requires a CameraSensor.\n";
  }
  // clang-format on
  GazeboNode::configure(this->sensor->WorldName());

  // load camera
  this->camera = this->sensor->Camera();
  if (!this->sensor) {
    gzerr << "CameraGPlugin not attached to a camera sensor!\n";
    return;
  }

  // keep track of image width, height and depth
  this->seq = 0;
  this->image_width = this->camera->ImageWidth();
  this->image_height = this->camera->ImageHeight();
  this->image_depth = this->camera->ImageDepth();
  this->image_format = this->camera->ImageFormat();
  // this->image = cv::Mat(this->image_width, this->image_height, CV_8UC3);

  // register callback CameraGPlugin::onNewFrame()
  // clang-format off
  this->update_conn = this->camera->ConnectNewImageFrame(
    std::bind(
      &CameraGPlugin::onNewFrame,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4,
      std::placeholders::_5
    )
  );
  // clang-format on

  // setup publisher
  this->registerPublisher<gazebo::msgs::Image>(IMAGE_PUB_TOPIC);
}

void CameraGPlugin::onNewFrame(const unsigned char *image_raw,
                               const int image_width,
                               const int image_height,
                               const int image_depth,
                               const std::string &format) {
  UNUSED(image_depth);

  // clang-format off
  gazebo::common::Image image;
  image.SetFromData(
    image_raw,
    image_width,
    image_height,
    image.ConvertPixelFormat(format)
  );
  // clang-format on

  gazebo::msgs::Image msg;
  gazebo::msgs::Set(&msg, image);
  this->gaz_pubs[IMAGE_PUB_TOPIC]->Publish(msg);
}

GZ_REGISTER_SENSOR_PLUGIN(CameraGPlugin)
} // namespace gaz
} // end of atl namepspace
