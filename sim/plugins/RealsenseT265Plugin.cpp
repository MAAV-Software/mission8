#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <random>

#include <Eigen/Dense>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/global_update_t.hpp>
#include <common/messages/visual_odometry_t.hpp>
#include <gnc/utils/ZcmConversion.hpp>
#include <zcm/zcm-cpp.hpp>

using ignition::math::Pose3d;
using zcm::ZCM;

namespace gazebo
{
class T265CameraPlugin : public ModelPlugin
{
public:
    T265CameraPlugin() : last_time_(0), frequency(30), zcm("ipc"), rd{}, gen{rd()} {}

    void Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
        world_ = model->GetWorld();
        model_ = model;
        link_ = model_->GetLinks()[0];

        name_ = model_->GetName();

        starting_pose_ = link_->WorldPose();
        starting_pose_inv_ = starting_pose_.Inverse();

        last_time_ = world_->SimTime();

        updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&T265CameraPlugin::OnUpdate, this));

        std::cout << "[T265 Plugin] Loaded!" << std::endl;
    }

    void add_noise() {}

    void OnUpdate()
    {
        auto current_time = world_->SimTime();
        uint64_t usec = current_time.sec * 1000000;
        usec += current_time.nsec / 1000;
        double dt = (current_time - last_time_).Double();

        bool should_publish = dt >= 1 / frequency;

        if (should_publish)
        {
            // Get perfect faked pose
            Pose3d current_pose = link_->WorldPose();

            Pose3d measured_odom = last_pose_.Inverse() * current_pose;
            Pose3d measured_pose = starting_pose_inv_ * current_pose;

            // Set times
            visual_odom_msg_.utime = usec;
            global_update_msg_.utime = usec;

            // Convert to NED
            // Convert Visual Odometry
            visual_odom_msg_.rotation.data[0] = measured_odom.Rot().W();
            visual_odom_msg_.rotation.data[1] = measured_odom.Rot().Y();
            visual_odom_msg_.rotation.data[2] = measured_odom.Rot().X();
            visual_odom_msg_.rotation.data[3] = -measured_odom.Rot().Z();

            visual_odom_msg_.translation.data[0] = measured_odom.Pos().Y();
            visual_odom_msg_.translation.data[1] = measured_odom.Pos().X();
            visual_odom_msg_.translation.data[2] = -measured_odom.Pos().Z();

            // Convert Global Update
            global_update_msg_.attitude.data[0] = measured_pose.Rot().W();
            global_update_msg_.attitude.data[1] = measured_pose.Rot().Y();
            global_update_msg_.attitude.data[2] = measured_pose.Rot().X();
            global_update_msg_.attitude.data[3] = -measured_pose.Rot().Z();

            global_update_msg_.position.data[0] = measured_pose.Pos().Y();
            global_update_msg_.position.data[1] = measured_pose.Pos().X();
            global_update_msg_.position.data[2] = -measured_pose.Pos().Z();

            zcm.publish(maav::SIM_VISUAL_ODOMETRY_CHANNEL, &visual_odom_msg_);
            zcm.publish(maav::SIM_GLOBAL_UPDATE_CHANNEL, &global_update_msg_);

            add_noise();

            zcm.publish(maav::VISUAL_ODOMETRY_CHANNEL, &visual_odom_msg_);
            zcm.publish(maav::GLOBAL_UPDATE_CHANNEL, &global_update_msg_);

            last_pose_ = current_pose;
            last_time_ = current_time;
        }
    }

private:
    gazebo::physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    std::string name_;

    Pose3d starting_pose_;
    Pose3d starting_pose_inv_;
    Pose3d last_pose_;

    event::ConnectionPtr updateConnection;

    common::Time last_time_;
    common::Time last_publish_time_;
    double frequency;

    visual_odometry_t visual_odom_msg_;
    global_update_t global_update_msg_;

    sdf::ElementPtr sdf;
    ZCM zcm;

    std::random_device rd;
    std::mt19937 gen;

    int count;
};  // namespace gazebo

GZ_REGISTER_MODEL_PLUGIN(T265CameraPlugin)
}  // namespace gazebo