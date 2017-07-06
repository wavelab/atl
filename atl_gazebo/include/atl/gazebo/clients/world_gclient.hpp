#ifndef atl_GAZEBO_WORLD_CLIENT_HPP
#define atl_GAZEBO_WORLD_CLIENT_HPP

#include <fstream>
#include <streambuf>
#include <string>

#include <Eigen/Dense>

#include "atl/gazebo/gazebo_node.hpp"
#include "atl/utils/math.hpp"

namespace atl {
namespace gaz {

// MESSAGES TYPES
#define TIME_MSG gazebo::msgs::Time
#define MODEL_MSG gazebo::msgs::Model
#define MODEL_POSE_MSG atl_msgs::msgs::ModelPose
#define FACTORY_MSG gazebo::msgs::Factory
#define REQUEST_MSG gazebo::msgs::Request
#define WORLD_CONTROL_MSG gazebo::msgs::WorldControl
#define SERVER_CONTROL_MSG gazebo::msgs::ServerControl

// SUBSCRIBE TOPICS
#define WORLD_CLOCK_TOPIC "~/world/clock"
#define WORLD_STATS_TOPIC "~/world_stats"

// PUBLISH TOPICS
#define FACTORY_TOPIC "~/factory"
#define MODEL_REMOVE_TOPIC "~/model/remove"
#define MODEL_POSE_TOPIC "~/model/pose"
#define WORLD_CONTROL_TOPIC "~/world_control"
#define WORLD_LOAD_TOPIC "~/world/load"
#define WORLD_CLEAR_TOPIC "~/world/clear"
#define SERVER_CONTROL_TOPIC "/gazebo/server/control"

class GTime {
public:
  gazebo::msgs::Time sim_time;
  gazebo::msgs::Time pause_time;
  gazebo::msgs::Time real_time;
  bool paused;
  long int iterations;

  GTime(void){};
};

class WorldGClient : public GazeboNode {
public:
  bool connected;
  GTime time;

  WorldGClient(void);
  ~WorldGClient(void);
  int configure(void);
  int shutdownServer(void);
  int pauseWorld(void);
  int unPauseWorld(void);
  int resetWorld(void);
  int loadWorld(std::string file_path);
  int clearWorld(void);
  int loadModel(std::string model_name);
  int loadModel(std::string model_name,
                Eigen::Vector3d pos,
                Eigen::Quaterniond quat);
  int removeModel(std::string model_name);
  int setModelPose(std::string model_name, Vec3 pos, Vec3 rpy);
  virtual void clockCallback(ConstTimePtr &msg);
  virtual void worldStatsCallback(ConstWorldStatisticsPtr &msg);
};

}  // namespace gaz
}  // namespace atl
#endif
