#ifndef _SIM_PLUGIN__
#define _SIM_PLUGIN__

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


namespace gazebo
{
    class QuadrotorPlugin: public ModelPlugin
    {
    public:
        gazebo::physics::ModelPtr model;
        gazebo::event::ConnectionPtr update_conn;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr subcriber;

        gazebo::math::Pose pose;

        QuadrotorPlugin(void);
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        void OnUpdate(const common::UpdateInfo &info);
    };

    QuadrotorPlugin::QuadrotorPlugin(void)
    {
        printf("loading [libquadrotor_plugin.so]!\n");
    }

    void QuadrotorPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        this->model = model;
        this->update_conn = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&QuadrotorPlugin::OnUpdate, this, _1)
        );
    }

    void QuadrotorPlugin::OnUpdate(const common::UpdateInfo &info)
    {

        this->model->SetLinearVel(gazebo::math::Vector3(0.0, 0.0, 0.0));
        this->pose = this->model->GetWorldPose();
    }

    GZ_REGISTER_MODEL_PLUGIN(QuadrotorPlugin)
}
#endif
