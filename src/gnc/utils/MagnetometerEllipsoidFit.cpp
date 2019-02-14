#include <gnc/utils/MagnetometerEllipsoidFit.hpp>
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::SelfAdjointEigenSolver;
using Eigen::Vector3d;
using Eigen::VectorXd;

using namespace std;

namespace maav
{
namespace gnc
{
MagParams runMagnetometerCalibration(const VectorXd& x, const VectorXd& y, const VectorXd& z)
{
    // Algorithm from
    // https://github.com/martindeegan/drone/blob/master/copter/matlab/ellipsoid_fit.m

    MatrixXd D(x.size(), 9);

    for (int i = 0; i < (int)x.size(); i++)
    {
        D(i, 0) = (x(i) * x(i));
        D(i, 1) = (y(i) * y(i));
        D(i, 2) = (z(i) * z(i));
        D(i, 3) = 2 * x(i) * y(i);
        D(i, 4) = 2 * z(i) * x(i);
        D(i, 5) = 2 * y(i) * z(i);
        D(i, 6) = 2 * x(i);
        D(i, 7) = 2 * y(i);
        D(i, 8) = 2 * z(i);
    }

    MatrixXd v;

    v = (D.transpose() * D)
            .colPivHouseholderQr()
            .solve(D.transpose() * Eigen::MatrixXd::Ones(D.rows(), 1));

    Matrix4d A;
    A(0, 0) = v(0);
    A(0, 1) = v(3);
    A(0, 2) = v(4);
    A(0, 3) = v(6);
    A(1, 0) = v(3);
    A(1, 1) = v(1);
    A(1, 2) = v(5);
    A(1, 3) = v(7);
    A(2, 0) = v(4);
    A(2, 1) = v(5);
    A(2, 2) = v(2);
    A(2, 3) = v(8);
    A(3, 0) = v(6);
    A(3, 1) = v(7);
    A(3, 2) = v(8);
    A(3, 3) = -1;

    Vector3d v_col;
    v_col << v(6), v(7), v(8);

    Vector3d offset = -A.block(0, 0, 3, 3).colPivHouseholderQr().solve(v_col);

    MatrixXd Tmtx = MatrixXd::Identity(4, 4);

    Tmtx(3, 0) = offset(0);
    Tmtx(3, 1) = offset(1);
    Tmtx(3, 2) = offset(2);

    Matrix4d AT = Tmtx * A * Tmtx.transpose();

    SelfAdjointEigenSolver<MatrixXd> es((AT.block(0, 0, 3, 3) / -AT(3, 3)));

    MatrixXd ev = es.eigenvalues().real();

    MatrixXd rotM = es.eigenvectors().real();

    double scale_x = sqrt(std::abs(1 / ev(0, 0)));
    double scale_y = sqrt(std::abs(1 / ev(1, 0)));
    double scale_z = sqrt(std::abs(1 / ev(2, 0)));

    Vector3d scale(scale_x, scale_y, scale_z);

    return {offset, scale, rotM};
}
}  // namespace gnc
}  // namespace maav