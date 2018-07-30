#pragma once

#include <Eigen/Dense>

#include "state/base_state.hpp"

namespace maav {
namespace gnc {
namespace kalman {

/**
 * Attitude: Represented as an element of SO3
 * Angular rate: 3D vector
 * Position: 3D vector
 * Velocity: 3D vector
 *
 * Estimation TODO:
 * Gyro bias
 * Accel bias
 * Gravity vector
 * Magnetic field vector
 */
class KalmanState : public state::BaseState {
   public:
    constexpr static size_t DIM = 16;
    constexpr static size_t E_DIM = DIM - 1;

    using CovarianceMatrix = Eigen::Matrix<double, E_DIM, E_DIM>;
    using ErrorStateVector = Eigen::Matrix<double, E_DIM, 1>;

   public:
    KalmanState(uint64_t time_usec);

    static KalmanState zero(uint64_t time_usec);

    const Eigen::Vector3d& gyro_bias() const;
    Eigen::Vector3d& gyro_bias();

    const Eigen::Vector3d& accel_bias() const;
    Eigen::Vector3d& accel_bias();

    const Eigen::Vector3d& gravity_vector() const;
    Eigen::Vector3d& gravity_vector();

    const Eigen::Vector3d& magnetic_field_vector() const;
    Eigen::Vector3d& magnetic_field_vector();

    const CovarianceMatrix& covariance() const;
    CovarianceMatrix& covariance();

   protected:
    Eigen::Vector3d _gyro_bias;

    Eigen::Vector3d _accel_bias;

    Eigen::Vector3d _gravity;

    Eigen::Vector3d _magnetic_field;

    CovarianceMatrix _covar;
};

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
