#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Vector3d;

struct MagParams
{
    Vector3d offset;
    Vector3d scale;
    MatrixXd rotM;
};