
#include <iostream>

#include <common/utils/yaml_matrix.hpp>
#include <gnc/kalman/Extrinsics.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
Extrinsics::Extrinsics(YAML::Node config)
    : rot_(Eigen::Quaterniond(config["rot"].as<Eigen::Vector4d>())),
      pos_(config["pos"].as<Eigen::Vector3d>())
{
}

State Extrinsics::sensorState(const State& state) const
{
    State sensor_state = state;
    sensor_state.position() += state.attitude() * position();
    sensor_state.attitude() *= rotation();
    return sensor_state;
}

State Extrinsics::imuState(const State& state) const
{
    State imu_state = state;
    imu_state.attitude() *= rotation().inverse();
    imu_state.position() -= imu_state.attitude() * position();
    return imu_state;
}

const Sophus::SO3d& Extrinsics::rotation() const { return rot_; }
const Eigen::Vector3d& Extrinsics::position() const { return pos_; }
}
}
}