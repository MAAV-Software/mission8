#pragma once

#include <Eigen/Dense>

#include "state/base_state.hpp"

namespace maav {
namespace gnc {
namespace kalman {

// TODO: Increase dimension as we add more estimated states
constexpr size_t STATE_DIM = 16;
constexpr size_t ERROR_STATE_DIM = STATE_DIM - 1;

typedef Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>
    CovarianceMatrix;

typedef Eigen::Matrix<double, ERROR_STATE_DIM, 1> ErrorStateVector;

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
    KalmanState(uint64_t time_usec);

    KalmanState zero(uint64_t time_usec);

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
