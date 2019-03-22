
#include <gazebo/common/Exception.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/sensors/SensorManager.hh>

#include "PlaneFitSensor.hpp"
#include "PlaneFitSensorPrivate.hpp"

#include <common/messages/MsgChannels.hpp>
#include <common/messages/plane_fit_t.hpp>

using namespace gazebo;
using namespace sensors;

/////////////////////////////////////////////////
PlaneFitSensor::PlaneFitSensor() : Sensor(sensors::OTHER), dataPtr(new PlaneFitSensorPrivate)
{
    this->active = false;
}

/////////////////////////////////////////////////
PlaneFitSensor::~PlaneFitSensor() {}

/////////////////////////////////////////////////
void PlaneFitSensor::Load(const std::string& _worldName, sdf::ElementPtr _sdf)
{
    Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void PlaneFitSensor::Load(const std::string& _worldName)
{
    Sensor::Load(_worldName);

    std::cout << "Loaded PlaneFit sensor" << std::endl;
    // std::cout << "Parent: " << this->ParentName() << std::endl;
    this->dataPtr->entity = this->world->EntityByName(this->ParentName());
}

/////////////////////////////////////////////////
void PlaneFitSensor::Fini()
{
    Sensor::Fini();
    this->dataPtr->entity.reset();
}

//////////////////////////////////////////////////
void PlaneFitSensor::Init() { Sensor::Init(); }

//////////////////////////////////////////////////
bool PlaneFitSensor::UpdateImpl(const bool /*_force*/)
{
    const auto& pose = this->dataPtr->entity->WorldPose();
    const auto& velocity = this->dataPtr->entity->WorldLinearVel();

    this->dataPtr->msg.z.data[0] = -pose.Pos().Z();
    this->dataPtr->msg.z_dot.data[0] = -velocity.Z();
    this->dataPtr->msg.roll.data[0] = pose.Rot().Roll();
    this->dataPtr->msg.pitch.data[0] = -pose.Rot().Pitch();

    auto time = this->world->SimTime();
    uint64_t usec = time.sec * 1000000;
    usec += time.nsec / 1000;
    this->dataPtr->msg.utime = usec;

    // TODO: Add noise
    this->dataPtr->zcm.publish(maav::SIM_PLANE_FIT_CHANNEL, &(this->dataPtr->msg));
    this->dataPtr->zcm.publish(maav::PLANE_FIT_CHANNEL, &(this->dataPtr->msg));
    return true;
}
