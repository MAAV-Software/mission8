#pragma once

namespace maav {
namespace gnc {
namespace measurements {

/*
 * Stores the necessary info we need from a lidar message.
 */
struct LidarMeasurement {
	double distance;
	uint64_t time;
}

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
