#include "atl_gazebo/plugins/box_gplugin.hpp"

namespace atl {
namespace gaz {

BoxGPlugin::BoxGPlugin(void) {
  printf("LOADING [libbox_gplugin.so]!\n");
}

void BoxGPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  UNUSED(sdf);

  // store the pointer to the model
  this->model = model;

  // listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&BoxGPlugin::onUpdate, this, _1));
}

void BoxGPlugin::onUpdate(const gazebo::common::UpdateInfo &info) {
  UNUSED(info);

  this->model->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.1));
  this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.2));
}

GZ_REGISTER_MODEL_PLUGIN(BoxGPlugin)
}  // namespace gaz
}  // namespace atl
