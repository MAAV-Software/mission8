#pragma once

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <common/messages/global_update_t.hpp>
#include <zcm/zcm-cpp.hpp>

namespace gazebo {
namespace sensors {
/// \internal
/// \brief RFID tag private data.
class SlamSensorPrivate {
    /// \brief Pointer the entity that has the SlamSensor
   public:
    physics::EntityPtr entity;

    global_update_t msg;

    zcm::ZCM zcm{"ipc"};
};
}
}