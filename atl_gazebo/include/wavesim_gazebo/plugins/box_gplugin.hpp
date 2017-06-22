#ifndef WAVESIM_GAZEBO_BOX_PLUGIN_HPP
#define WAVESIM_GAZEBO_BOX_PLUGIN_HPP

#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>


namespace wavesim {
namespace gaz {

class BoxGPlugin : public gazebo::ModelPlugin {
public:
  gazebo::physics::ModelPtr model;
  gazebo::event::ConnectionPtr updateConnection;

  BoxGPlugin(void);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void onUpdate(const gazebo::common::UpdateInfo &info);
};

}  // end of gaz namespace
}  // end of wavesim namespace
#endif
