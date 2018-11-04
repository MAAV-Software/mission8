#include "gnc/state/base_state.hpp"
#include "gnc/constants.hpp"

namespace maav
{
namespace gnc
{
namespace state
{
BaseState::BaseState(uint64_t time_usec) : _time_usec(time_usec) {}
BaseState BaseState::zero(uint64_t time_usec)
{
    BaseState new_state(time_usec);
    new_state.attitude() = Sophus::SO3d(Eigen::Quaterniond::Identity());
    new_state.angular_velocity() = Eigen::Vector3d::Zero();
    new_state.position() = Eigen::Vector3d::Zero();
    new_state.velocity() = Eigen::Vector3d::Zero();
    new_state.acceleration() = Eigen::Vector3d::Zero();
    return new_state;
}

const Sophus::SO3d& BaseState::attitude() const { return _attitude; }
Sophus::SO3d& BaseState::attitude() { return _attitude; }
const Eigen::Vector3d& BaseState::angular_velocity() const { return _angular_velocity; }
Eigen::Vector3d& BaseState::angular_velocity() { return _angular_velocity; }
const Eigen::Vector3d& BaseState::position() const { return _position; }
Eigen::Vector3d& BaseState::position() { return _position; }
const Eigen::Vector3d& BaseState::velocity() const { return _velocity; }
Eigen::Vector3d& BaseState::velocity() { return _velocity; }
const Eigen::Vector3d& BaseState::acceleration() const { return _acceleration; }
Eigen::Vector3d& BaseState::acceleration() { return _acceleration; }
uint64_t BaseState::time_usec() const { return _time_usec; }
double BaseState::time_sec() const
{
    return static_cast<double>(_time_usec) * constants::USEC_TO_SEC;
}

}  // namespace state
}  // namespace gnc
}  // namespace maav
