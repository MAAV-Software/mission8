#pragma once

#include <Eigen/Dense>
#include "sophus/so3.hpp"

namespace maav {
namespace gnc {
namespace measurements {

/*
 * Stores the necessary info we need from a global update (SLAM) measurement
 */
struct GlobalUpdateMeasurement {
	Eigen::Vector3d position;
	Sophus::SO3d attitude;
	uint64_t time;
}

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
