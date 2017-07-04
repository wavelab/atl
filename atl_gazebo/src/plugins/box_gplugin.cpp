#include "atl_gazebo/plugins/box_gplugin.hpp"

namespace atl {
namespace gaz {

BoxGPlugin::BoxGPlugin(void) {
  printf("LOADING [libbox_gplugin.so]!\n");
}

void BoxGPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  // store the pointer to the model
  this->model = model;

  // listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&BoxGPlugin::onUpdate, this, _1));
}

void BoxGPlugin::onUpdate(const gazebo::common::UpdateInfo &info) {
  this->model->SetLinearVel(gazebo::math::Vector3(0.0, 0.0, 0.1));
  this->model->SetAngularVel(gazebo::math::Vector3(0.0, 0.0, 0.2));
}

GZ_REGISTER_MODEL_PLUGIN(BoxGPlugin)
}  // end of gaz namespace
}  // end of atl namespace
