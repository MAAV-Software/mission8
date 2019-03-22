#include <cmath>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/common.hh>
#include <ignition/math.hh>

#include <common/messages/state_t.hpp>
#include <zcm/zcm-cpp.hpp>

#include <Eigen/Dense>

using zcm::ZCM;

namespace gazebo
{
class GroundTruthPlugin : public WorldPlugin
{
    /// \brief Constructor
public:
    GroundTruthPlugin() : zcm("ipc"), last_time()
    {
        std::cout << "Ground Truth Plugin works!" << std::endl;
    }
    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _world A pointer to the world that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
    void OnUpdate()
    {
        auto sim_time = world->SimTime();
        auto diff = sim_time - last_time;
        const common::Time period(0, 1000000000 / 100);
        if ((sim_time - last_time) >= period)
        {
            const ignition::math::Pose3d& pose = model->WorldPose();
            const ignition::math::Vector3<double>& position = pose.Pos();

            const ignition::math::Vector3d& velocity = model->WorldLinearVel();
            const ignition::math::Vector3d& acceleration = model->WorldLinearAccel();
            const ignition::math::Vector3d& angular_velocity = model->RelativeAngularVel();

            // Convert position from ENU to NED (Y to X, X to Y, -Z to Z)
            state.position.data[0] = position.Y();
            state.position.data[1] = position.X();
            state.position.data[2] = -position.Z();

            // Gives correct heading
            ignition::math::Quaternion<double> q = pose.Rot();
            ignition::math::Quaternion<double> x_rot{cos(M_PI / 2), sin(M_PI / 2), 0, 0};
            ignition::math::Quaternion<double> z_rot{cos(-M_PI / 2 / 2), 0, 0, sin(-M_PI / 2 / 2)};
            q = x_rot * z_rot * q;
            state.attitude.data[0] = q.W();
            state.attitude.data[1] = q.X();
            state.attitude.data[2] = q.Y();
            state.attitude.data[3] = q.Z();

            // Convert velocity from ENU to NED (Y to X, X to Y, -Z to Z)
            state.velocity.data[0] = velocity.Y();
            state.velocity.data[1] = velocity.X();
            state.velocity.data[2] = -velocity.Z();

            // I think these might be in the wrong frame (untested theory)
            state.angular_velocity.data[0] = angular_velocity.X();
            state.angular_velocity.data[1] = angular_velocity.Y();
            state.angular_velocity.data[2] = angular_velocity.Z();

            // TODO: figure out how estimator sends these!!
            // Convert acceleration from ENU to NED (Y to X, X to Y, -Z to Z)
            state.acceleration.data[0] = acceleration.Y();
            state.acceleration.data[1] = acceleration.X();
            state.acceleration.data[2] = -acceleration.Z();

            state.covariance.rows = 1;
            state.covariance.cols = 1;
            state.covariance.data = {{1}};

            uint64_t time = sim_time.sec * 1000000;
            time += sim_time.nsec / 1000;
            state.utime = time;
            last_time = sim_time;

            zcm.publish("SIM_STATE", &state);
        }
    }
    virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        world = _world;
        sdf = _sdf;

        model = world->ModelByName("matrice_100");

        this->updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&GroundTruthPlugin::OnUpdate, this));
    }

private:
    physics::WorldPtr world;
    physics::ModelPtr model;
    sdf::ElementPtr sdf;
    ZCM zcm;
    state_t state;
    common::Time last_time;

private:
    event::ConnectionPtr updateConnection;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_WORLD_PLUGIN(GroundTruthPlugin)
}  // namespace gazebo