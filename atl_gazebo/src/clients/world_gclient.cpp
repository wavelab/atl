#include "atl_gazebo/clients/world_gclient.hpp"


namespace atl {
namespace gaz {

WorldGClient::WorldGClient(void) {
  this->connected = false;
}

WorldGClient::~WorldGClient(void) {
  if (this->connected) {
    gazebo::client::shutdown();
  }
}

int WorldGClient::configure(void) {
  // pre-check
  if (this->connected) {
    // already connected
    return 0;

  } else {
    // connect
    this->connected = gazebo::client::setup(0, NULL);
    if (this->connected == false) {
      return -1;
    }
  }

  // setup gazebo node
  GazeboNode::configure();

  // publishers
  // clang-format off
  this->registerPublisher<FACTORY_MSG>(FACTORY_TOPIC);
  this->registerPublisher<REQUEST_MSG>(MODEL_REMOVE_TOPIC);
  this->registerPublisher<MODEL_POSE_MSG>(MODEL_POSE_TOPIC);
  this->registerPublisher<WORLD_CONTROL_MSG>(WORLD_CONTROL_TOPIC);
  this->registerPublisher<REQUEST_MSG>(WORLD_LOAD_TOPIC);
  this->registerPublisher<REQUEST_MSG>(WORLD_CLEAR_TOPIC);
  this->registerPublisher<SERVER_CONTROL_MSG>(SERVER_CONTROL_TOPIC);
  this->waitForConnection();
  // clang-format on

  // subscribers
  // clang-format off
  this->registerSubscriber(WORLD_CLOCK_TOPIC, &WorldGClient::clockCallback, this);
  this->registerSubscriber(WORLD_STATS_TOPIC, &WorldGClient::worldStatsCallback, this);
  // clang-format on

  return 0;
}

int WorldGClient::shutdownServer(void) {
  gazebo::msgs::ServerControl msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_stop(true);
  this->gaz_pubs[SERVER_CONTROL_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::pauseWorld(void) {
  gazebo::msgs::WorldControl msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_pause(true);
  this->gaz_pubs[WORLD_CONTROL_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::unPauseWorld(void) {
  gazebo::msgs::WorldControl msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_pause(false);
  this->gaz_pubs[WORLD_CONTROL_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::resetWorld(void) {
  gazebo::msgs::WorldControl msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.mutable_reset()->set_all(true);
  this->gaz_pubs[WORLD_CONTROL_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::loadWorld(std::string file_path) {
  std::ifstream sdf_file;
  std::stringstream buf;
  gazebo::msgs::Request msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // open and load sdf file into string
  sdf_file.open(file_path, std::ifstream::in);
  if (sdf_file.is_open() == false) {
    return -2;
  }
  buf << sdf_file.rdbuf();

  // publish message
  msg.set_id(0);
  msg.set_request("WORLD_LOAD");
  msg.set_data(buf.str());
  this->gaz_pubs[WORLD_LOAD_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::clearWorld(void) {
  gazebo::msgs::Request msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_id(0);
  msg.set_request("WORLD_CLEAR");
  this->gaz_pubs[WORLD_CLEAR_TOPIC]->Publish(msg);
  sleep(1);

  return 0;
}

int WorldGClient::loadModel(std::string model_name) {
  gazebo::msgs::Factory msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_sdf_filename("model://" + model_name);
  this->gaz_pubs[FACTORY_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::loadModel(std::string model_name,
                            Eigen::Vector3d pos,
                            Eigen::Quaterniond quat) {
  gazebo::msgs::Factory msg;
  ignition::math::Vector3d p;
  ignition::math::Quaterniond q;
  ignition::math::Pose3d pose;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // setup
  p = ignition::math::Vector3d(pos(0), pos(1), pos(2));
  q = ignition::math::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
  pose = ignition::math::Pose3d(p, q);

  // publish msg
  msg.set_sdf_filename("model://" + model_name);
  gazebo::msgs::Set(msg.mutable_pose(), pose);
  this->gaz_pubs[FACTORY_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::removeModel(std::string model_name) {
  gazebo::msgs::Request msg;

  // pre-check
  if (this->connected == false) {
    return -1;
  }

  // publish msg
  msg.set_id(0);
  msg.set_request("MODEL_REMOVE");
  msg.set_data(model_name);
  this->gaz_pubs[MODEL_REMOVE_TOPIC]->Publish(msg);

  return 0;
}

int WorldGClient::setModelPose(std::string model_name, Vec3 pos, Vec3 rpy) {
  atl_msgs::msgs::ModelPose msg;

  msg.set_model_name(model_name);
  msg.set_x(pos(0));
  msg.set_y(pos(1));
  msg.set_z(pos(2));
  msg.set_roll(rpy(0));
  msg.set_pitch(rpy(1));
  msg.set_yaw(rpy(2));
  this->gaz_pubs[MODEL_POSE_TOPIC]->Publish(msg);

  return 0;
}

void WorldGClient::clockCallback(ConstTimePtr &msg) {
  gazebo::msgs::Time time;

  time.set_sec(msg->sec());
  time.set_nsec(msg->nsec());

  this->time.sim_time = time;
}

void WorldGClient::worldStatsCallback(ConstWorldStatisticsPtr &msg) {
  // this->time.sim_time;  // set by the WorldGClient::clockCallback()
  this->time.sim_time = msg->sim_time();
  this->time.pause_time = msg->pause_time();
  this->time.real_time = msg->real_time();
  this->time.paused = msg->paused();
  this->time.iterations = msg->iterations();
}

}  // end of gaz namespace
}  // end of atl namespace
