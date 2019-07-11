#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <common/messages/matrix_t.hpp>
#include <common/messages/quaternion_t.hpp>
#include <common/messages/vector1_t.hpp>
#include <common/messages/vector2_t.hpp>
#include <common/messages/vector3_t.hpp>
#include <common/messages/vector4_t.hpp>

/*
    Conversions from zcm types to std::vectors for plotting
*/

template <class ZcmVector>
std::vector<double> convertVector(double time, const ZcmVector& vec)
{
    std::vector<double> converted(vec.dim + 1);
    converted[0] = time;
    for (size_t i = 0; i < vec.dim; i++)
    {
        converted[i + 1] = std::isfinite(vec.data[i]) ? vec.data[i] : 0;
    }
    return converted;
}

std::vector<double> convertQuaternion(double time, const quaternion_t& quat);

Eigen::MatrixXd convertMatrix(const matrix_t& mat);
