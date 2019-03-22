
#include <gazebo/common/Exception.hh>
#include <iostream>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/sensors/SensorManager.hh>

#include "SlamSensor.hpp"
#include "SlamSensorPrivate.hpp"

#include <common/messages/global_update_t.hpp>

using namespace gazebo;
using namespace sensors;

/////////////////////////////////////////////////
SlamSensor::SlamSensor() : Sensor(sensors::OTHER), dataPtr(new SlamSensorPrivate)
{
    this->active = false;
    std::ios::sync_with_stdio(false);
    std::cout << std::showpos << std::setprecision(4);
}

/////////////////////////////////////////////////
SlamSensor::~SlamSensor() {}

/////////////////////////////////////////////////
void SlamSensor::Load(const std::string& _worldName, sdf::ElementPtr _sdf)
{
    Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void SlamSensor::Load(const std::string& _worldName)
{
    Sensor::Load(_worldName);

    std::cout << "Loaded SlamSensor sensor" << std::endl;
    this->dataPtr->entity = this->world->EntityByName(this->ParentName());
}

/////////////////////////////////////////////////
void SlamSensor::Fini()
{
    Sensor::Fini();
    this->dataPtr->entity.reset();
}

//////////////////////////////////////////////////
void SlamSensor::Init() { Sensor::Init(); }

//////////////////////////////////////////////////
bool SlamSensor::UpdateImpl(const bool /*_force*/)
{
    const auto& pose = this->dataPtr->entity->WorldPose();

    auto time = this->world->SimTime();
    uint64_t usec = time.sec * 1000000;
    usec += time.nsec / 1000;
    this->dataPtr->msg.utime = usec;

    this->dataPtr->msg.position.data[0] = pose.Pos().Y();
    this->dataPtr->msg.position.data[1] = pose.Pos().X();
    this->dataPtr->msg.position.data[2] = -1 * pose.Pos().Z();

    const ignition::math::Quaterniond heading_fix(0, 0, -M_PI / 2);
    const ignition::math::Quaterniond q = pose.Rot() * heading_fix;

    this->dataPtr->msg.attitude.data[0] = q.W();
    this->dataPtr->msg.attitude.data[1] = q.Y();
    this->dataPtr->msg.attitude.data[2] = q.X();
    this->dataPtr->msg.attitude.data[3] = -q.Z();

    // std::cout << "X: " << dataPtr->msg.position[0]
    //           << " Y: " << dataPtr->msg.position[1]
    //           << " Z: " << dataPtr->msg.position[2] << '\n';
    // std::cout << "W: " << dataPtr->msg.attitude[0]
    //           << " X: " << dataPtr->msg.attitude[1]
    //           << " Y: " << dataPtr->msg.attitude[2]
    //           << " Z: " << dataPtr->msg.attitude[3] << std::endl;
    // TODO: Add noise

    this->dataPtr->zcm.publish("SIM_GLOBAL_UPDATE_CHANNEL", &(this->dataPtr->msg));
    return true;
}
