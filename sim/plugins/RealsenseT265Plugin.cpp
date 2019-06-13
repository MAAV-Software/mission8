#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <random>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/global_update_t.hpp>
#include <common/messages/visual_odometry_t.hpp>
#include <gnc/utils/ZcmConversion.hpp>
#include <zcm/zcm-cpp.hpp>
#include "common.hpp"

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
        link_ = model_->GetLink("T265::link");

        if (!link_)
        {
            std::cout << "[T265 Plugin] No link!" << std::endl;
        }

        name_ = model_->GetName();

        starting_pose_ = convertPoseToNED(link_->WorldPose());
        starting_pose_inverse_ = starting_pose_.inverse();
        last_pose_ = starting_pose_;

        last_time_ = world_->SimTime();

        updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&T265CameraPlugin::OnUpdate, this));

        std::cout << "[T265 Plugin] Loaded!" << std::endl;
    }

    void add_noise()
    {
        // TODO: add noise
    }

    void OnUpdate()
    {
        const auto current_time = world_->SimTime();
        uint64_t usec = current_time.sec * 1000000;
        usec += current_time.nsec / 1000;
        const double dt = (current_time - last_time_).Double();

        if (dt >= 1.0 / frequency)
        {
            // Get perfect faked pose
            const Sophus::SE3d current_sim_pose = convertPoseToNED(link_->WorldPose());

            // Get global pose relative to start
            const Sophus::SE3d pose = starting_pose_inverse_ * current_sim_pose;
            const Eigen::Vector3d position = pose.translation();
            const Sophus::SO3d attitude = pose.so3();

            // Get visual odometry measurement
            const Sophus::SE3d measured_odom = last_pose_.inverse() * current_sim_pose;
            const Eigen::Vector3d odom_translation = measured_odom.translation();
            const Sophus::SO3d odom_rotation = measured_odom.so3();

            // Set times
            visual_odom_msg_.utime = usec;
            global_update_msg_.utime = usec;

            // Convert Visual Odometry
            visual_odom_msg_.rotation = maav::gnc::convertQuaternion(odom_rotation);
            visual_odom_msg_.translation = maav::gnc::convertVector3d(odom_translation);

            // Convert Global Update
            global_update_msg_.attitude = maav::gnc::convertQuaternion(attitude);
            global_update_msg_.position = maav::gnc::convertVector3d(position);

            zcm.publish(maav::SIM_VISUAL_ODOMETRY_CHANNEL, &visual_odom_msg_);
            zcm.publish(maav::SIM_GLOBAL_UPDATE_CHANNEL, &global_update_msg_);

            add_noise();

            zcm.publish(maav::VISUAL_ODOMETRY_CHANNEL, &visual_odom_msg_);
            zcm.publish(maav::GLOBAL_UPDATE_CHANNEL, &global_update_msg_);

            last_pose_ = current_sim_pose;
            last_time_ = current_time;
        }
    }

private:
    gazebo::physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    std::string name_;

    Sophus::SE3d starting_pose_;
    Sophus::SE3d starting_pose_inverse_;
    Sophus::SE3d last_pose_;

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