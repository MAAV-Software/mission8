#include <gnc/control/MagParams.hpp>
#include <Eigen/Dense>

using Eigen::VectorXd;

namespace maav
{
namespace gnc
{
MagParams runMagnetometerCalibration(const VectorXd& x, const VectorXd& y, const VectorXd& z);
}
}  // namespace maav
