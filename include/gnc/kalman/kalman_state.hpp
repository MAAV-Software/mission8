#pragma once

#include <Eigen/Dense>

#include <gnc/state/base_state.hpp>

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
    constexpr static size_t DIM = 4 + 3 + 3;
    constexpr static size_t E_DIM = DIM - 1;

    using CovarianceMatrix = Eigen::Matrix<double, E_DIM, E_DIM>;
    using ErrorStateVector = Eigen::Matrix<double, E_DIM, 1>;

   public:
    explicit KalmanState(uint64_t time_usec);

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

   public:
    // TODO: Make private
    static KalmanState mean(const std::vector<KalmanState>& sigma_points,
                            const std::vector<double>& weights);

    static CovarianceMatrix cov(const KalmanState& mean,
                                const std::vector<KalmanState>& sigma_points,
                                const std::vector<double>& weights);

   public:
    static KalmanState compute_gaussian(const std::vector<KalmanState>& points,
                                        const std::vector<double>& m_weights,
                                        const std::vector<double>& c_weights);

    KalmanState& operator+=(const ErrorStateVector& e_state);
};

Sophus::SO3d weighted_average(const std::vector<Sophus::SO3d>& points,
                              const std::vector<double>& weights);

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
