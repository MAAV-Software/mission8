#include "ZcmConversion.hpp"

std::vector<double> convertQuaternion(double time, const quaternion_t& quat)
{
    Eigen::Quaterniond q{quat.data[0], quat.data[1], quat.data[2], quat.data[3]};
    double roll =
        atan2(2 * q.w() * q.x() + q.y() * q.z(), 1 - 2 * (q.x() * q.x() + 2 * q.y() * q.y()));
    double pitch = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    double yaw =
        -atan2(2 * q.z() * q.w() - 2 * q.x() * q.y(), 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z());

    return {time, roll, pitch, yaw};
}

Eigen::MatrixXd convertMatrix(const matrix_t& mat)
{
    Eigen::MatrixXd convertedMat(mat.rows, mat.cols);
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            convertedMat(i, j) = mat.data[i][j];
        }
    }
    return convertedMat;
}