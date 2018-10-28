#include <gnc/kalman/history.hpp>
#include <iterator>
#include <memory>

#include <gnc/kalman/history.hpp>

using maav::gnc::measurements::ImuMeasurement;
using maav::gnc::measurements::Measurement;
using maav::gnc::measurements::MeasurementSet;
using std::pair;

namespace maav
{
namespace gnc
{
namespace kalman
{
History::History(YAML::Node config)
	: _size(config["size"].as<size_t>()), tolerance(config["tolerance"].as<uint64_t>())
{
}

pair<History::Iterator, History::Iterator> History::add_measurement(MeasurementSet &measurements)
{
	// Insert zero state if empty
	if (_history.empty())
	{
		uint64_t start_time = measurements.imu->time_usec;
		Measurement start_measurement;
		start_measurement.imu = measurements.imu;
		KalmanState initial_state = KalmanState::zero(start_time);

		_history.emplace_back(initial_state, start_measurement);
		return {_history.end(), _history.end()};
	}

	// Insert IMU first
	uint64_t imu_time = measurements.imu->time_usec;
	Measurement imu_measurement;
	imu_measurement.imu = measurements.imu;
	_history.emplace_back(KalmanState(imu_time), imu_measurement);
	measurements.imu = nullptr;
	auto last_modified = std::prev(_history.end());

	// Microsecond tolerance to merge measurements
	// IMU will be running on a 10000 microsecond period

	if (measurements.lidar)
	{
		uint64_t lidar_time = measurements.lidar->time_usec;
		auto snap_iter = find_snapshot(lidar_time);
		if (snap_iter != _history.end())
		{
			snap_iter->measurement.lidar = measurements.lidar;
			measurements.lidar = nullptr;

			last_modified = set_last_modified(last_modified, snap_iter);
		}
	}

	if (measurements.plane_fit)
	{
		uint64_t plane_fit_time = measurements.plane_fit->time_usec;
		auto snap_iter = find_snapshot(plane_fit_time);
		if (snap_iter != _history.end())
		{
			snap_iter->measurement.plane_fit = measurements.plane_fit;
			measurements.plane_fit = nullptr;

			last_modified = set_last_modified(last_modified, snap_iter);
		}
	}

	if (measurements.visual_odometry)
	{
		uint64_t vo_time = measurements.visual_odometry->time_usec;
		auto snap_iter = find_snapshot(vo_time);
		if (snap_iter != _history.end())
		{
			snap_iter->measurement.visual_odometry = measurements.visual_odometry;
			measurements.visual_odometry = nullptr;

			last_modified = set_last_modified(last_modified, snap_iter);
		}
	}

	if (measurements.global_update)
	{
		uint64_t gu_time = measurements.global_update->time_usec;
		auto snap_iter = find_snapshot(gu_time);
		if (snap_iter != _history.end())
		{
			snap_iter->measurement.global_update = measurements.global_update;
			measurements.global_update = nullptr;

			last_modified = set_last_modified(last_modified, snap_iter);
		}
	}

	if (last_modified != _history.begin())
	{
		std::advance(last_modified, -1);
	}
	resize(last_modified);

	return {last_modified, _history.end()};
}

void History::resize(Iterator last_modified)
{
	while (_history.size() > _size)
	{
		auto iter = _history.begin();
		if (iter == last_modified)
		{
			break;
		}
		else
		{
			_history.pop_front();
		}
	}
}

History::Iterator History::find_snapshot(uint64_t time)
{
	auto end = _history.rbegin();
	if (time > end->get_time())
	{
		if (time - end->get_time() < tolerance)
		{
			// If within tolerance of last IMU, add it
			return std::prev(_history.end());
		}
		else
		{
			// Otherwise, it is too new and IMU needs to be interpolated
			return _history.end();
		}
	}

	for (auto r_iter = std::next(end); r_iter != _history.rend(); std::advance(r_iter, 1))
	{
		uint64_t snapshot_time = r_iter->get_time();
		if (time > snapshot_time)
		{
			auto next_iter = r_iter.base();
			auto prev_iter = std::prev(next_iter);

			uint64_t next_t = next_iter->get_time();
			uint64_t prev_t = prev_iter->get_time();

			// Check tolerances and insert interpolated IMU if necessary
			if (time - prev_t <= tolerance)
			{
				return prev_iter;
			}
			else if (next_t - time <= tolerance)
			{
				return next_iter;
			}
			else
			{
				ImuMeasurement interp_imu = interpolate_imu(*(prev_iter->measurement.imu),
															*(next_iter->measurement.imu), time);
				Measurement interp_measurement;
				interp_measurement.imu = std::make_shared<ImuMeasurement>(interp_imu);
				Snapshot interp_snapshot{KalmanState(time), interp_measurement};
				return _history.insert(next_iter, interp_snapshot);
			}
		}
	}

	// If older than oldest measurement, discard
	return _history.end();
}

ImuMeasurement History::interpolate_imu(const ImuMeasurement &prev, const ImuMeasurement &next,
										uint64_t interp_time) const
{
	ImuMeasurement interpolated = prev;
	uint64_t prev_t = prev.time_usec;
	uint64_t next_t = next.time_usec;

	double weight = static_cast<double>(interp_time - prev_t);
	weight /= static_cast<double>(next_t - prev_t);

	interpolated.time_usec = interp_time;
	interpolated.angular_rates += weight * (next.angular_rates - prev.angular_rates);
	interpolated.acceleration += weight * (next.acceleration - prev.acceleration);
	interpolated.magnetometer += weight * (next.magnetometer - prev.magnetometer);

	return interpolated;
}
History::Iterator History::set_last_modified(const History::Iterator last_modified,
											 const History::Iterator modified) const
{
	if (modified->get_time() < last_modified->get_time())
	{
		return modified;
	}
	else
	{
		return last_modified;
	}
}
size_t History::size() { return _history.size(); }
}  // namespace kalman
}  // namespace gnc
}  // namespace maav