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
#include <gnc/utils/ZcmConversion.hpp>
#include <zcm/zcm-cpp.hpp>
#include "common.hpp"

#include <Eigen/Eigen>
#include <sophus/se3.hpp>
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
        auto world = model->GetWorld();

        // clang-format off
        std::cout << "===============================================================================" << std::endl;
        std::cout << "============================== *** EXTRINSICS *** =============================" << std::endl;
        std::cout << "===============================================================================" << std::endl;
        fuselage_link_ = parent_->GetLink("fuselage");
        Sophus::SE3d matrice_pose = convertPoseToNED(fuselage_link_->WorldPose());
        std::cout << "Matrice Pose" << std::endl;
        std::cout << matrice_pose.matrix() << std::endl << std::endl;

        physics::LinkPtr forward_camera =
            world->ModelByName("MaavMatrice::Forward Camera")->GetLink("D435::link");
        if (forward_camera)
        {
            std::cout << "============================" << std::endl;
            std::cout << "=== ** Forward Camera ** ===" << std::endl;
            std::cout << "============================" << std::endl;
            Sophus::SE3d pose = convertPoseToNED(forward_camera->WorldPose());
            Sophus::SE3d extrinsics = matrice_pose.inverse() * pose;
            std::cout << "Extrinsics ----------------------" << std::endl;
            std::cout << extrinsics.matrix() << std::endl;
            std::cout << "Extrinsics Log ------------------" << std::endl;
            std::cout << extrinsics.log().transpose() << std::endl;
        }
        std::cout << std::endl;

        physics::LinkPtr downward_camera =
            world->ModelByName("MaavMatrice::Downward Camera")->GetLink("D435::link");
        if (downward_camera)
        {
            std::cout << "============================" << std::endl;
            std::cout << "=== ** Downward Camera ** ==" << std::endl;
            std::cout << "============================" << std::endl;
            Sophus::SE3d pose = convertPoseToNED(downward_camera->WorldPose());
            Sophus::SE3d extrinsics = matrice_pose.inverse() * pose;
            std::cout << "Extrinsics ----------------------" << std::endl;
            std::cout << extrinsics.matrix() << std::endl;
            std::cout << "Extrinsics Log ------------------" << std::endl;
            std::cout << extrinsics.log().transpose() << std::endl;
        }
        std::cout << std::endl;

        physics::LinkPtr slam_camera =
            world->ModelByName("MaavMatrice::SLAM Camera")->GetLink("T265::link");
        if (slam_camera)
        {
            std::cout << "============================" << std::endl;
            std::cout << "===== ** SLAM Camera ** ====" << std::endl;
            std::cout << "============================" << std::endl;
            Sophus::SE3d pose = convertPoseToNED(slam_camera->WorldPose());
            Sophus::SE3d extrinsics = matrice_pose.inverse() * pose;
            std::cout << "Extrinsics ----------------------" << std::endl;
            std::cout << extrinsics.matrix() << std::endl;
            std::cout << "Extrinsics Log ------------------" << std::endl;
            std::cout << extrinsics.log().transpose() << std::endl;
        }
        std::cout << std::endl;

        physics::LinkPtr lidar =
            world->ModelByName("MaavMatrice::Height Lidar")->GetLink("LidarLite::link");
        if (lidar)
        {
            std::cout << "============================" << std::endl;
            std::cout << "======= ** Lidar ** ========" << std::endl;
            std::cout << "============================" << std::endl;
            Sophus::SE3d pose = convertPoseToNED(lidar->WorldPose());
            Sophus::SE3d extrinsics = matrice_pose.inverse() * pose;
            std::cout << "Extrinsics ----------------------" << std::endl;
            std::cout << extrinsics.matrix() << std::endl;
            std::cout << "Extrinsics Log ------------------" << std::endl;
            std::cout << extrinsics.log().transpose() << std::endl;
        }
        std::cout << std::endl;

        std::cout << "===============================================================================" << std::endl;
        std::cout << "===============================================================================" << std::endl;
        std::cout << "===============================================================================" << std::endl;

        // clang-format on

        updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&MaavVehiclePlugin::OnUpdate, this));

        starting_pose_ = matrice_pose;
        starting_pose_inverse_ = starting_pose_.inverse();

        std::cout << "[Vehicle Plugin] Loaded!" << std::endl;
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
        if (dt >= 1.0 / UPDATE_RATE)
        {
            // Update previous time
            last_time_ = usec;

            // Get ground truth and publish as a state message
            msg_.utime = usec;

            // Get and convert pose/velocity/accel/angvel
            const Sophus::SE3d current_sim_pose = convertPoseToNED(fuselage_link_->WorldPose());
            const Sophus::SE3d pose = starting_pose_inverse_ * current_sim_pose;

            const Eigen::Vector3d position = pose.translation();
            const Eigen::Vector3d velocity =
                starting_pose_inverse_.so3() * convertVectorToNED(parent_->WorldLinearVel());
            const Eigen::Vector3d acceleration =
                starting_pose_inverse_.so3() * convertVectorToNED(parent_->WorldLinearAccel());
            const Eigen::Vector3d angular_velocity =
                starting_pose_inverse_.so3() * convertVectorToNED(parent_->WorldAngularVel());

            const Sophus::SO3d attitude = pose.so3();
            msg_.attitude = maav::gnc::convertQuaternion(attitude);
            msg_.position = maav::gnc::convertVector3d(position);
            msg_.velocity = maav::gnc::convertVector3d(velocity);
            msg_.acceleration = maav::gnc::convertVector3d(acceleration);
            msg_.angular_velocity = maav::gnc::convertVector3d(angular_velocity);

            zcm_.publish(maav::GT_INERTIAL_CHANNEL, &msg_);
        }
    }

public:
    virtual void OnNewDepthFrame() const {}

private:
    physics::ModelPtr parent_;
    physics::LinkPtr fuselage_link_;
    std::string name_;

    zcm::ZCM zcm_;
    uint64_t last_time_;
    groundtruth_inertial_t msg_;

    event::ConnectionPtr updateConnection;

    Sophus::SE3d starting_pose_;
    Sophus::SE3d starting_pose_inverse_;
};

GZ_REGISTER_MODEL_PLUGIN(MaavVehiclePlugin)

}  // namespace gazebo
