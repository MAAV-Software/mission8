#ifndef EXTRINSICS_HPP
#define EXTRINSICS_HPP

#include <yaml-cpp/yaml.h>
#include <sophus/so3.hpp>

#include <gnc/State.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * @class Represents a relative pose of a sensor with respect to the IMU.
 *
 * State state of our quadcopter is represented by the state of our IMU. In order to correctly add
 * corrections to the state, our sensor model must take into account its relative pose.
 */
class Extrinsics
{
public:
    Extrinsics(YAML::Node config);

    /**
     * @brief Relative attitude of the sensor to the IMU
     */
    const Sophus::SO3d& rotation() const;

    /**
     * @brief Relative position of the sensor to the IMU
     */
    const Eigen::Vector3d& position() const;

    /**
     * @brief Computes the state (world frame) of the sensor given the state of the IMU
     * @param state IMU state
     */
    State sensorState(const State& state) const;

    /**
     * @brief Computes the state (world frame) of the IMU given the state of the sensor
     * @param state Sensor state
     */
    State imuState(const State& state) const;

private:
    const Sophus::SO3d rot_;
    const Eigen::Vector3d pos_;
};
}
}
}

#endif