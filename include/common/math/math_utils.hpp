#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <cmath>
#include "Eigen/Core"

namespace maav
{
inline Eigen::Matrix3d rotMatZ(double angle)
{
	Eigen::Matrix3d R;
	R << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1;
	return R;
}

inline Eigen::Matrix3d rotMatX(double angle)
{
	Eigen::Matrix3d R;
	R << 1, 0, 0, 0, std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle);
	return R;
}

inline Eigen::Matrix3d rotMatY(double angle)
{
	Eigen::Matrix3d R;
	R << std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle);
	return R;
}

inline Eigen::Vector3d rotZ(const Eigen::Vector3d& vec, double angle)
{
	return rotMatZ(angle) * vec;
}

inline Eigen::Vector3d rotX(const Eigen::Vector3d& vec, double angle)
{
	return rotMatX(angle) * vec;
}

inline Eigen::Vector3d rotY(const Eigen::Vector3d& vec, double angle)
{
	return rotMatY(angle) * vec;
}

/**
 * @brief Returns bounded input between upper and lower values
 * @param input Value to bound
 * @param upper	Upper bound
 * @param lower Lower bound
 */
inline double bounded(double input, double upper, double lower)
{
	if (input > upper) return upper;
	if (input < lower) return lower;
	return input;
}
}

#endif  // math_utils.hpp
