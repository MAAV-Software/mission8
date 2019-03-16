#include "ZcmConversion.hpp"

std::vector<double> convertQuaternion(double time, const quaternion_t& quat)
{
    return convertVector(time, quat);
}

std::vector<double> convertMatrix(double time, const matrix_t& mat)
{
    return std::vector<double>({time, 1, 2, 3});
}