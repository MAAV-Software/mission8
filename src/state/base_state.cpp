#include "base_state.hpp"

namespace maav {
namespace gnc {
namespace state {

BaseState::BaseState() {}

BaseState BaseState::zero() {
    BaseState new_state;
    return new_state;
}

const Sophus::SO3d& BaseState::get_attitude() const { return attitude; }

Sophus::SO3d& BaseState::get_attitude() { return attitude; }

const Eigen::Vector3d& BaseState::get_angular_velocity() const {
    return angular_velocity;
}
Eigen::Vector3d& BaseState::get_angular_velocity() { return angular_velocity; }

const Eigen::Vector3d& BaseState::get_position() const { return position; }

Eigen::Vector3d& BaseState::get_position() { return position; }

const Eigen::Vector3d& BaseState::get_velocity() const { return velocity; }

Eigen::Vector3d& BaseState::get_velocity() { return velocity; }

uint64_t BaseState::get_time_usec() const { return time_usec; }

double BaseState::get_time_sec() const {
    constexpr double USEC_PER_SEC = 1000000.0;
    return static_cast<double>(time_usec) / USEC_PER_SEC;
}

}  // namespace state
}  // namespace gnc
}  // namespace maav
