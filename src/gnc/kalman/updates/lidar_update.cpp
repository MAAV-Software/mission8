#include <gnc/kalman/updates/lidar_update.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
LidarMeasurement LidarUpdate::predicted(const KalmanState& state)
{
	// TODO: Implement
	LidarMeasurement meas;
	meas.distance = 0;
	return meas;
}

LidarMeasurement LidarUpdate::measured(const measurements::Measurement& meas)
{
	// TODO: Implement
	LidarMeasurement measured_reading;
	measured_reading.distance = meas.lidar->distance;
	return measured_reading;
}

LidarMeasurement::ErrorStateVector LidarMeasurement::operator-(const LidarMeasurement& other) const
{
	LidarMeasurement::ErrorStateVector ret;
	ret(0) = distance - other.distance;
	return ret;
}

LidarMeasurement& LidarMeasurement::operator+=(
	const LidarMeasurement::ErrorStateVector& error_state)
{
	distance += error_state(0);
	return *this;
}

const LidarMeasurement::CovarianceMatrix& LidarMeasurement::covariance() const
{
	return _covariance;
}
LidarMeasurement::CovarianceMatrix& LidarMeasurement::covariance() { return _covariance; }
}
}
}