#include "atl/gazebo/plugins/world_gplugin.hpp"

namespace atl {
namespace gaz {

WorldGPlugin::WorldGPlugin() { printf("LOADING [libworld_gplugin.so]!\n"); }

void WorldGPlugin::Load(gazebo::physics::WorldPtr parent, sdf::ElementPtr sdf) {
  UNUSED(sdf);

  this->world_sdf = sdf;
  this->world = parent;
  this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WorldGPlugin::onUpdate, this, _1));

  // setup gazebo node
  // clang-format off
  GazeboNode::configure();
  this->registerPublisher<gazebo::msgs::Time>(WORLD_CLOCK_TOPIC);
  this->registerSubscriber(MODEL_REMOVE_TOPIC, &WorldGPlugin::modelRemoveCallback, this);
  this->registerSubscriber(MODEL_POSE_TOPIC, &WorldGPlugin::modelPoseCallback, this);
  this->registerSubscriber(WORLD_LOAD_TOPIC, &WorldGPlugin::worldLoadCallback, this);
  this->registerSubscriber(WORLD_CLEAR_TOPIC, &WorldGPlugin::worldClearCallback, this);
  // clang-format on
}

void WorldGPlugin::onUpdate(const gazebo::common::UpdateInfo &info) {
  gazebo::msgs::Time msg;

  msg.set_sec(info.simTime.sec);
  msg.set_nsec(info.simTime.nsec);
  this->gaz_pubs[WORLD_CLOCK_TOPIC]->Publish(msg);
}

void WorldGPlugin::modelRemoveCallback(ConstRequestPtr &msg) {
  gazebo::physics::ModelPtr model;

  std::cout << "REMOVING MODEL [" << msg->data() << "]!" << std::endl;

  // METHOD: 1
  // model = this->world->ModelByName(msg->data());
  // model->Fini();

  // METHOD: 2
  // this->world->RemoveModel(msg->data());
}

void WorldGPlugin::modelPoseCallback(ModelPosePtr &msg) {
  gazebo::physics::ModelPtr model;
  ignition::math::Pose3d pose;

  model = this->world->ModelByName(msg->model_name());
  pose = model->WorldPose();
  pose.Pos().X() = msg->x();
  pose.Pos().Y() = msg->y();
  pose.Pos().Z() = msg->z();
  model->SetWorldPose(pose);
}

void WorldGPlugin::worldLoadCallback(ConstRequestPtr &msg) {
  sdf::SDF tmp;
  sdf::SDF sdf_file;
  sdf::ElementPtr world;
  sdf::ElementPtr model;

  // log message
  std::cout << "LOADING WORLD ..." << std::endl;

  // parse world file
  sdf_file.SetFromString(msg->data());
  world = sdf_file.Root()->GetElement("world");

  // load first model
  model = world->GetElement("model");
  tmp.Root(model);
  this->world->InsertModelSDF(tmp);

  // load subsequent models
  model = model->GetNextElement("model");
  while (model != NULL) {
    tmp.Root(model);
    this->world->InsertModelSDF(tmp);
    model = model->GetNextElement("model");
  }
}

void WorldGPlugin::worldClearCallback(ConstRequestPtr &msg) {
  UNUSED(msg);
  std::cout << "CLEARING WORLD ..." << std::endl;
  this->world->Clear();
}

GZ_REGISTER_WORLD_PLUGIN(WorldGPlugin)
} // namespace gaz
} // namespace atl
