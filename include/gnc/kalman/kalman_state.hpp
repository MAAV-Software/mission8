#pragma once

#include <Eigen/Dense>

#include <gnc/state/base_state.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
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
class KalmanState : public state::BaseState
{
   public:
    constexpr static size_t DoF = 9;

    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, 1>;

   public:
    KalmanState(uint64_t time_usec);

    KalmanState() = default;

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
    constexpr static size_t N = 1 + 2 * DoF;
    // TODO: Make private
    static KalmanState mean(
        const std::array<KalmanState, N>& sigma_points, const std::array<double, N>& weights);

    static CovarianceMatrix cov(const KalmanState& mean,
        const std::array<KalmanState, N>& sigma_points, const std::array<double, N>& weights);

    /**
     * Functions required for computing with gaussians and such.
     */
   public:
    static KalmanState compute_gaussian(const std::array<KalmanState, 1 + 2 * DoF>& points,
        const std::array<double, N>& m_weights, const std::array<double, N>& c_weights);

    KalmanState& operator+=(const ErrorStateVector& e_state);

    ErrorStateVector operator-(const KalmanState& other) const;
};

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
