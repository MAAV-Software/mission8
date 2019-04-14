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
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <string>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <zcm/zcm-cpp.hpp>

namespace gazebo
{
class MaavPlanefitPlugin : public ModelPlugin
{
public:
    MaavPlanefitPlugin() : node("ipc"), last_time(0) {}

public:
    ~MaavPlanefitPlugin() {}

public:
    virtual void Load(boost::shared_ptr<gazebo::physics::Model> model, sdf::ElementPtr sdf)
    {
        parent_ = model;

        updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&MaavPlanefitPlugin::OnUpdate, this));
    }

public:
    void OnUpdate()
    {
        auto time = parent_->GetWorld()->SimTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;
        msg.utime = usec;
        uint64_t dt = usec - last_time;
        if (dt < 1000000 / 15) return;

        last_time = usec;
        ignition::math::Pose3d offset;
        offset.Set(0, 0, 0, 0, -1.57, 0);
        auto pose = parent_->GetLink("D435::link")->WorldPose();
        pose = offset * pose;
        double z = -pose.Pos().Z();
        msg.z_dot.data[0] = (z - msg.z.data[0]) / static_cast<double>(dt) * 1000000.0;
        msg.z.data[0] = z;

        double roll = pose.Rot().Roll();
        double pitch = pose.Rot().Pitch();
        msg.roll.data[0] = roll;
        msg.pitch.data[0] = pitch;

        node.publish(maav::SIM_PLANE_FIT_CHANNEL, &msg);
        node.publish(maav::PLANE_FIT_CHANNEL, &msg);
    }

public:
    virtual void OnNewDepthFrame() const {}

private:
    physics::ModelPtr parent_;
    zcm::ZCM node;
    plane_fit_t msg;
    uint64_t last_time;

    event::ConnectionPtr updateConnection;
};

GZ_REGISTER_MODEL_PLUGIN(MaavPlanefitPlugin)
}  // namespace gazebo
