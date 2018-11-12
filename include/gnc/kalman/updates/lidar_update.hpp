#ifndef LIDAR_UPDATE_HPP
#define LIDAR_UPDATE_HPP

#include <yaml-cpp/yaml.h>

#include <gnc/kalman/base_update.hpp>
#include <gnc/measurements/Lidar.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * TODO: Combine this with maav::gnc::measurements::Lidar
 * Represents the TargetSpace of the lidar update
 */
class SensorMeasurement
{
    public:
    /**
     * Necessary definitions
     */
    constexpr static size_t DoF = 1;
    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, 1>;
    using SensorVector = Eigen::Matrix<double, 1, 1>;

    /**
     * @brief Computes the error state between two SensorMeasurements
     */
    ErrorStateVector operator-(const SensorMeasurement& other) const;

    /**
     * @brief adds the error state into a SensorMeasurement
     */
    SensorMeasurement& operator+=(const ErrorStateVector& other);

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
    const SensorVector& distance() const;

    /**
     * @return Mutable reference to the measured value
     */
    SensorVector& distance();

    /**
     * @brief Computes the mean and covariance of a set of SensorMeasurements
     * @param points Sigma points from an UnscentedTransform
     * @param m_weights Mean weights from an UnscentedTransform
     * @param c_weights Covariance weights from an UnscentedTransform
     */
    static SensorMeasurement compute_gaussian(
        const std::array<SensorMeasurement, KalmanState::N>& points,
        const std::array<double, KalmanState::N>& m_weights,
        const std::array<double, KalmanState::N>& c_weights)
    {
        SensorMeasurement gaussian;
        gaussian.distance() = SensorVector::Zero();
        for (size_t i = 0; i < KalmanState::N; i++)
        {
            gaussian += m_weights[i] * points[i].distance();
        }

        gaussian.covariance() = CovarianceMatrix::Zero();
        for (size_t i = 0; i < KalmanState::N; i++)
        {
            gaussian.covariance() +=
                c_weights[i] * (points[i] - gaussian) * (points[i] - gaussian).transpose();
        }

        return gaussian;
    }

    private:
    SensorVector distance_;
    CovarianceMatrix covariance_;
};

/**
 * A correction step for a downward facing lidar on a flat surface
 */
class LidarUpdate : public BaseUpdate<SensorMeasurement>
{
    public:
    /**
     *  @param config This yaml node must have a "lidar" key with unscented transform parameters and
     * a sensor covariance matrix, R
     */
    LidarUpdate(YAML::Node config);

    /**
     * @breif Computes the observation model (h(x)) for a state provided by an UnscentedTransform
     * @param state A sigma point proved by an UnscentedTransform
     * @return The predicted lidar observation
     */
    SensorMeasurement predicted(const KalmanState& state);

    /**
     * @param meas
     * @return The relevant lidar measurement from the list of measurements
     */
    SensorMeasurement measured(const measurements::Measurement& meas);

    /**
     * @brief Performs the correction step for a lidar
     * @param snapshot A mutable reference to a point in time. The state will be updated according
     * to the sensor model
     */
    void operator()(History::Snapshot& snapshot);

    private:
    using BaseUpdate<SensorMeasurement>::correct;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace maav

#endif