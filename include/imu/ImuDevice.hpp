#pragma once

#include <common/messages/imu_t.hpp>

namespace maav
{
/**
 * Abstract interface to an IMU
 */
class ImuDevice
{
public:
    virtual void read(imu_t& msg) = 0;
};
}