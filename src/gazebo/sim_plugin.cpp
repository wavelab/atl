#ifndef _SIM_PLUGIN__
#define _SIM_PLUGIN__

#include <gazebo/gazebo.hh>


namespace gazebo
{
    class WorldPluginTutorial : public WorldPlugin
    {
    public:
        WorldPluginTutorial():WorldPlugin()
        {
            printf("Hello World!\n");
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {

        }
    };

    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
#endif
