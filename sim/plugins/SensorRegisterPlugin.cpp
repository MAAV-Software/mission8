#include <PlaneFitSensor.hpp>
#include <gazebo/gazebo.hh>
#include "SlamSensor.hpp"

namespace gazebo
{
class RegisterMySensorPlugin : public SystemPlugin
{
    /////////////////////////////////////////////
    /// \brief Destructor
public:
    virtual ~RegisterMySensorPlugin() {}

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
public:
    void Load(int /*_argc*/, char** /*_argv*/)
    {
        // RegisterPlaneFitSensor();
        // RegisterSlamSensor();
        printf("Loaded custom sensors!\n");
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
private:
    void Init() {}
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(RegisterMySensorPlugin)
}
