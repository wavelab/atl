#ifndef atl_GAZEBO_WORLD_PLUGIN_HPP
#define atl_GAZEBO_WORLD_PLUGIN_HPP

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "atl/gazebo/gazebo_node.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gaz {

#define MODEL_REMOVE_TOPIC "~/model/remove"
#define MODEL_POSE_TOPIC "~/model/pose"
#define WORLD_LOAD_TOPIC "~/world/load"
#define WORLD_CLEAR_TOPIC "~/world/clear"
#define WORLD_CLOCK_TOPIC "~/world/clock"

class WorldGPlugin : public gazebo::WorldPlugin, public GazeboNode {
public:
  sdf::ElementPtr world_sdf;
  gazebo::physics::WorldPtr world;
  gazebo::event::ConnectionPtr update_conn;

  WorldGPlugin(void);
  void Load(gazebo::physics::WorldPtr parent, sdf::ElementPtr sdf);
  void onUpdate(const gazebo::common::UpdateInfo &info);
  void modelRemoveCallback(ConstRequestPtr &msg);
  void modelPoseCallback(ModelPosePtr &msg);
  void worldLoadCallback(ConstRequestPtr &msg);
  void worldClearCallback(ConstRequestPtr &msg);
};

}  // namespace gaz
}  // namespace atl
#endif
