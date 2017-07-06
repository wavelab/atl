#ifndef atl_GAZEBO_BOX_PLUGIN_HPP
#define atl_GAZEBO_BOX_PLUGIN_HPP

#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "atl/utils/utils.hpp"


namespace atl {
namespace gaz {

class BoxGPlugin : public gazebo::ModelPlugin {
public:
  gazebo::physics::ModelPtr model;
  gazebo::event::ConnectionPtr updateConnection;

  BoxGPlugin(void);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void onUpdate(const gazebo::common::UpdateInfo &info);
};

}  // namespace gaz
}  // namespace atl
#endif
