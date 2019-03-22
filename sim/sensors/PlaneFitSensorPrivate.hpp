#pragma once

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <common/messages/plane_fit_t.hpp>
#include <zcm/zcm-cpp.hpp>

namespace gazebo
{
namespace sensors
{
/// \internal
/// \brief RFID tag private data.
class PlaneFitSensorPrivate
{
    /// \brief Pointer the entity that has the PlaneFitSensor
public:
    physics::EntityPtr entity;

    plane_fit_t msg;

    zcm::ZCM zcm{"ipc"};
};
}
}
