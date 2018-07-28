#pragma once

#include <cstdint>

#include <Eigen/Dense>
#include "sophus/so3.hpp"

namespace maav {
namespace gnc {
namespace state {
/**
 * BaseState represents the state values needed by users of the localizer.
 * State values are represented in the North East Down coordinate system.
 * Orientations and angular velocities use right hand rule.
 */
class BaseState {
   public:
    /**
     * Create a state with uninitialized members
     */
    BaseState();

    /**
     * Initialize the state to identity values
     */
    static BaseState zero();

    /**
     * Returns a constant reference to the rotation matrix
     */
    const Sophus::SO3d& get_attitude() const;
    /**
     * Returns a mutable reference to the rotation matrix
     */
    Sophus::SO3d& get_attitude();

    const Eigen::Vector3d& get_angular_velocity() const;
    Eigen::Vector3d& get_angular_velocity();

    const Eigen::Vector3d& get_position() const;
    Eigen::Vector3d& get_position();

    const Eigen::Vector3d& get_velocity() const;
    Eigen::Vector3d& get_velocity();

    uint64_t get_time_usec() const;
    double get_time_sec() const;

   protected:
    uint64_t time_usec;

    /**
     * Orientation of the vehicle represented as a member of SO3
     */
    Sophus::SO3d attitude;

    Eigen::Vector3d angular_velocity;

    Eigen::Vector3d position;

    Eigen::Vector3d velocity;
};

}  // namespace state
}  // namespace gnc
}  // namespace maav
