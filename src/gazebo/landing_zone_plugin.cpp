#ifndef _SIM_PLUGIN__
#define _SIM_PLUGIN__

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


namespace gazebo
{
    class LandingZonePlugin: public ModelPlugin
    {
    public:
        gazebo::physics::ModelPtr model;
        gazebo::event::ConnectionPtr update_conn;

        LandingZonePlugin(void);
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        void OnUpdate(const common::UpdateInfo &info);
    };

    LandingZonePlugin::LandingZonePlugin(void)
    {
        printf("loading [liblanding_zone_plugin.so]!\n");
    }

    void LandingZonePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        this->model = model;
        this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&LandingZonePlugin::OnUpdate, this, _1)
        );
    }

    void LandingZonePlugin::OnUpdate(const common::UpdateInfo &info)
    {
        this->model->SetLinearVel(gazebo::math::Vector3(0.8, 0.0, 0.0));
    }

    GZ_REGISTER_MODEL_PLUGIN(LandingZonePlugin)
}
#endif
