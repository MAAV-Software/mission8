#ifndef GLOBAL_UPDATE_MEASUREMENT
#define GLOBAL_UPDATE_MEASUREMENT

#include <ostream>

#include <Eigen/Dense>
#include <sophus/so3.hpp>

#include <gnc/State.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
/*
 * Stores the necessary info we need from a global update (SLAM) measurement
 */
class GlobalUpdateMeasurement
{
public:
    /**
     * Necessary definitions
     */
    constexpr static size_t DoF = 6;
    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, 1>;

    /**
     * @brief Computes the error state between two SensorMeasurements
     */
    ErrorStateVector operator-(const GlobalUpdateMeasurement& other) const;

    /**
     * @brief adds the error state into a GlobalUpdateMeasurement
     */
    GlobalUpdateMeasurement& operator+=(const ErrorStateVector& other);

    /**
     * @return Const reference to the covariance matrix
     */
    const CovarianceMatrix& covariance() const;

    /**
     * @return Mutable reference to the covariance matrix
     */
    CovarianceMatrix& covariance();

    /**
     * @return Const reference to the measured value
     */
    const Sophus::SO3d& attitude() const;

    /**
     * @return Mutable reference to the measured value
     */
    Sophus::SO3d& attitude();

    const Eigen::Vector3d& position() const;

    Eigen::Vector3d& position();

    /**
     * @brief Computes the mean and covariance of a set of SensorMeasurements
     * @param points Sigma points from an UnscentedTransform
     * @param m_weights Mean weights from an UnscentedTransform
     * @param c_weights Covariance weights from an UnscentedTransform
     */
    static GlobalUpdateMeasurement compute_gaussian(
        const std::array<GlobalUpdateMeasurement, State::N>& points,
        const std::array<double, State::N>& m_weights,
        const std::array<double, State::N>& c_weights)
    {
        GlobalUpdateMeasurement gaussian;
        gaussian.attitude() = points[0].attitude();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian.position() += m_weights[i] * points[i].position();
        }

        gaussian.covariance() = CovarianceMatrix::Zero();
        for (size_t i = 0; i < State::N; i++)
        {
            gaussian.covariance() +=
                c_weights[i] * (points[i] - gaussian) * (points[i] - gaussian).transpose();
        }

        return gaussian;
    }

    uint64_t timeUSec() const;
    void setTime(uint64_t time_usec);

private:
    Eigen::Vector3d position_;
    Sophus::SO3d attitude_;
    uint64_t time_usec_;
    CovarianceMatrix covariance_;
};

std::ostream& operator<<(std::ostream& os, const GlobalUpdateMeasurement& meas);

}  // namespace measurements
}  // namespace gnc
}  // namespace maav

#endif
