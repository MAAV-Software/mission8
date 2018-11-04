#pragma once

#include <cstdint>

#include <Eigen/Dense>
#include "sophus/so3.hpp"

namespace maav
{
namespace gnc
{
namespace state
{
/**
 * BaseState represents the state values needed by users of the localizer.
 * State values are represented in the North East Down coordinate system.
 * Orientations and angular velocities use right hand rule.
 */
class BaseState
{
   public:
    /**
     * Create a state with uninitialized members
     */
    BaseState(uint64_t time_usec);

    BaseState() = default;

    /**
     * Initialize the state to identity values
     */
    static BaseState zero(uint64_t time_usec);

    /**
     * Returns a constant reference to the rotation matrix
     */
    const Sophus::SO3d& attitude() const;
    /**
     * Returns a mutable reference to the rotation matrix
     */
    Sophus::SO3d& attitude();

    const Eigen::Vector3d& angular_velocity() const;
    Eigen::Vector3d& angular_velocity();

    const Eigen::Vector3d& position() const;
    Eigen::Vector3d& position();

    const Eigen::Vector3d& velocity() const;
    Eigen::Vector3d& velocity();

    const Eigen::Vector3d& acceleration() const;
    Eigen::Vector3d& acceleration();

    uint64_t time_usec() const;
    double time_sec() const;

   protected:
    uint64_t _time_usec;

    /**
     * Orientation of the vehicle represented as a member of SO3
     */
    Sophus::SO3d _attitude;

    Eigen::Vector3d _angular_velocity;

    Eigen::Vector3d _position;

    Eigen::Vector3d _velocity;

    Eigen::Vector3d _acceleration;
};

}  // namespace state
}  // namespace gnc
}  // namespace maav
