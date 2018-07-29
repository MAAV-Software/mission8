#include "kalman_state.hpp"
#include "constants.hpp"

namespace maav {
namespace gnc {
namespace kalman {

KalmanState::KalmanState(uint64_t time_usec) : BaseState(time_usec) {}

KalmanState KalmanState::zero(uint64_t time_usec) {
    KalmanState new_state(time_usec);
    new_state.attitude() = Sophus::SO3d(Eigen::Quaterniond::Identity());
    new_state.angular_velocity() = Eigen::Vector3d::Zero();
    new_state.position() = Eigen::Vector3d::Zero();
    new_state.velocity() = Eigen::Vector3d::Zero();
    new_state.gyro_bias() = Eigen::Vector3d::Zero();
    new_state.accel_bias() = Eigen::Vector3d::Zero();
    new_state.gravity_vector() = {0.0, 0.0, -constants::STANDARD_GRAVITY};
    new_state.magnetic_field_vector() = {1.0, 0.0, 0.0};
    return new_state;
}

const Eigen::Vector3d& KalmanState::gyro_bias() const { return _gyro_bias; }

Eigen::Vector3d& KalmanState::gyro_bias() { return _gyro_bias; }

const Eigen::Vector3d& KalmanState::accel_bias() const { return _accel_bias; }

Eigen::Vector3d& KalmanState::accel_bias() { return _accel_bias; }

const Eigen::Vector3d& KalmanState::gravity_vector() const { return _gravity; }

Eigen::Vector3d& KalmanState::gravity_vector() { return _gravity; }

const Eigen::Vector3d& KalmanState::magnetic_field_vector() const {
    return _magnetic_field;
}

Eigen::Vector3d& KalmanState::magnetic_field_vector() {
    return _magnetic_field;
}

const CovarianceMatrix& KalmanState::covariance() const { return _covar; }

CovarianceMatrix& KalmanState::covariance() { return _covar; }

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
