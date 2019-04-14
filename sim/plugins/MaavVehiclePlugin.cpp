/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <string>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <zcm/zcm-cpp.hpp>

#include <Eigen/Eigen>
#include <sophus/so3.hpp>

namespace gazebo
{
class MaavVehiclePlugin : public ModelPlugin
{
public:
    MaavVehiclePlugin() : zcm_("ipc"), last_time_(0) {}

public:
    ~MaavVehiclePlugin() {}

public:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        parent_ = model;
        name_ = parent_->GetName();

        auto imu = sdf->GetElement("imu");
        if (imu)
        {
            std::string imu_model_name = imu->Get<std::string>("model_name");
            physics::ModelPtr imu_model = parent_->NestedModel(imu_model_name);

            if (imu_model)
                std::cout << "[VehiclePlugin] Found IMU at " << imu_model->GetName() << std::endl;
        }

        updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&MaavVehiclePlugin::OnUpdate, this));

        starting_pose_ = parent_->WorldPose();
        inverse_rot_ = starting_pose_.Inverse().Rot();

        std::cout << "[VehiclePlugin] Loaded!" << std::endl;
    }

public:
    void OnUpdate()
    {
        // Microseconds per second
        constexpr double USEC_PER_SEC = 1e6;
        // Hz
        constexpr double UPDATE_RATE = 100;

        // Convert Gazebo time format to microseconds
        auto time = parent_->GetWorld()->SimTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;

        double dt = static_cast<double>(usec - last_time_) / USEC_PER_SEC;
        if (dt >= 1 / UPDATE_RATE)
        {
            // Update previous time
            last_time_ = usec;

            // Get ground truth and publish as a state message
            msg_.utime = usec;

            // Get and convert pose/velocity/accel/angvel
            ignition::math::Pose3d pose = starting_pose_.Inverse() * parent_->WorldPose();
            ignition::math::Quaterniond attitude = pose.Rot();
            ignition::math::Vector3d position = pose.Pos();
            ignition::math::Vector3d velocity = inverse_rot_ * parent_->WorldLinearVel();
            ignition::math::Vector3d acceleration = inverse_rot_ * parent_->WorldLinearAccel();
            ignition::math::Vector3d angular_velocity = inverse_rot_ * parent_->WorldAngularVel();

            msg_.attitude.data[0] = attitude.W();
            msg_.attitude.data[1] = attitude.Y();
            msg_.attitude.data[2] = attitude.X();
            msg_.attitude.data[3] = -attitude.Z();

            msg_.position.data[0] = position.Y();
            msg_.position.data[1] = position.X();
            msg_.position.data[2] = -position.Z();

            msg_.velocity.data[0] = velocity.Y();
            msg_.velocity.data[1] = velocity.X();
            msg_.velocity.data[2] = -velocity.Z();

            msg_.acceleration.data[0] = acceleration.Y();
            msg_.acceleration.data[1] = acceleration.X();
            msg_.acceleration.data[2] = -acceleration.Z();

            msg_.angular_velocity.data[0] = angular_velocity.Y();
            msg_.angular_velocity.data[1] = angular_velocity.X();
            msg_.angular_velocity.data[2] = -angular_velocity.Z();

            zcm_.publish(maav::GT_INERTIAL_CHANNEL, &msg_);
        }
    }

public:
    virtual void OnNewDepthFrame() const {}

private:
    physics::ModelPtr parent_;
    std::string name_;

    zcm::ZCM zcm_;
    uint64_t last_time_;
    groundtruth_inertial_t msg_;

    event::ConnectionPtr updateConnection;

    ignition::math::Pose3d starting_pose_;
    ignition::math::Quaterniond inverse_rot_;
};

GZ_REGISTER_MODEL_PLUGIN(MaavVehiclePlugin)

}  // namespace gazebo
