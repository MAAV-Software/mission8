#include <gnc/kalman/updates/lidar_update.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
using ErrorStateVector = SensorMeasurement::ErrorStateVector;
using CovarianceMatrix = SensorMeasurement::CovarianceMatrix;
using SensorVector = SensorMeasurement::SensorVector;

ErrorStateVector SensorMeasurement::operator-(const SensorMeasurement& other) const
{
    return distance_ - other.distance_;
}

SensorMeasurement& SensorMeasurement::operator+=(const ErrorStateVector& other)
{
    distance_ += other;
    return *this;
}

const CovarianceMatrix& SensorMeasurement::covariance() const { return covariance_; }
CovarianceMatrix& SensorMeasurement::covariance() { return covariance_; }
const SensorVector& SensorMeasurement::distance() const { return distance_; }
SensorVector& SensorMeasurement::distance() { return distance_; }
LidarUpdate::LidarUpdate(YAML::Node config) : BaseUpdate(config["lidar"]) {}
SensorMeasurement LidarUpdate::predicted(const KalmanState& state)
{
    /*
        Here, we predict the length of a lidar beam given our state. With some trigonometry, you can
       observe that the length of a beam going down is -z / cos(theta). We rotate a unit z vector
       and dot it with the unit z vector to find cos(theta)
    */
    SensorMeasurement predicted_measurement;
    const Eigen::Vector3d vertical_vec = Eigen::Vector3d::UnitZ();
    double cos_theta = (state.attitude() * vertical_vec).dot(vertical_vec);
    predicted_measurement.distance()(0) = -state.position().z() / cos_theta;
    return predicted_measurement;
}

SensorMeasurement LidarUpdate::measured(const measurements::Measurement& meas)
{
    SensorMeasurement sensor_measurement;
    sensor_measurement.distance()(0) = meas.lidar->distance;
    return sensor_measurement;
}

void LidarUpdate::operator()(History::Snapshot& snapshot)
{
    // Check the validity of the lidar measurements
    if (!snapshot.measurement.lidar) return;
    double distance = snapshot.measurement.lidar->distance;
    if (std::isfinite(distance))
    {
        correct(snapshot);
    }
}
}  // namespace kalman
}  // namespace gnc
}  // namespace maav