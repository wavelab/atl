#include "wavesim_gazebo/plugins/df_camera_gplugin.hpp"

namespace wavesim {
namespace gaz {

DFCameraGPlugin::DFCameraGPlugin(void) {
  printf("LOADING [libdf_camera_gplugin.so]!\n");
}

void DFCameraGPlugin::Load(gazebo::sensors::SensorPtr sptr,
                           sdf::ElementPtr sdf) {
  // load sensor pointer
  // clang-format off
  this->sensor = std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(sptr);
  if (!this->sensor) {
    gzerr << "DFCameraGPlugin requires a CameraSensor.\n";
  }
  // clang-format on

  GazeboNode::configure(this->sensor->WorldName());

  // load camera
  this->camera = this->sensor->Camera();
  if (!this->sensor) {
    gzerr << "DFCameraGPlugin not attached to a camera sensor!\n";
    return;
  }

  // keep track of image width, height and depth
  this->seq = 0;
  this->image_width = this->camera->ImageWidth();
  this->image_height = this->camera->ImageHeight();
  this->image_depth = this->camera->ImageDepth();
  this->image_format = this->camera->ImageFormat();
  this->image = cv::Mat(this->image_width, this->image_height, CV_8UC3);

  // register callback DFCameraGPlugin::onNewFrame()
  // clang-format off
  this->update_conn = this->camera->ConnectNewImageFrame(
    std::bind(
      &DFCameraGPlugin::onNewFrame,
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

void DFCameraGPlugin::onNewFrame(const unsigned char *image,
                                 const int image_width,
                                 const int image_height,
                                 const int image_depth,
                                 const std::string &format) {
  int buf_size;
  unsigned char *data;
  gazebo::common::Image input_image;
  gazebo::msgs::Image pub_image;

  // setup
  buf_size = image_width * image_height * image_depth;
  data = new unsigned char[buf_size + 1];

  // convert binary data to cv::Mat.clone() is used to deep copy data to
  // this->image, // otherwise the mat data becomes void when called from
  // the ros plugin
  memcpy(data, image, buf_size);
  this->image = cv::Mat(image_height, image_width, CV_8UC3, data).clone();

  // publish image
  // clang-format off
  input_image.SetFromData(
    image,
    image_width,
    image_height,
    input_image.ConvertPixelFormat(format)
  );
  // clang-format on
  gazebo::msgs::Set(&pub_image, input_image);
  this->gaz_pubs[IMAGE_PUB_TOPIC]->Publish(pub_image);

  // debug
  // cv::imshow("DFCameraGPlugin Image", this->image);
  // cv::waitKey(1);

  // clean up
  delete data;
}

GZ_REGISTER_SENSOR_PLUGIN(DFCameraGPlugin)
}  // end of gaz namespace
}  // end of wavesim namepspace
