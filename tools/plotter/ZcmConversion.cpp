#include "ZcmConversion.hpp"

std::vector<double> convertQuaternion(double time, const quaternion_t& quat)
{
    return convertVector(time, quat);
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