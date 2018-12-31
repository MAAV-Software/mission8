#include <gnc/measurements/GlobalUpdateMeasurement.hpp>

namespace maav
{
namespace gnc
{
namespace measurements
{
using ErrorStateVector = GlobalUpdateMeasurement::ErrorStateVector;
using CovarianceMatrix = GlobalUpdateMeasurement::CovarianceMatrix;

ErrorStateVector GlobalUpdateMeasurement::operator-(const GlobalUpdateMeasurement& other) const
{
    ErrorStateVector difference;
    difference.segment<3>(0) = (attitude().inverse() * other.attitude()).log();
    difference.segment<3>(3) = (position() - other.position());
    return difference;
}

GlobalUpdateMeasurement& GlobalUpdateMeasurement::operator+=(const ErrorStateVector& other)
{
    attitude() *= Sophus::SO3d::exp(other.segment<3>(0));
    position() += other.segment<3>(3);
    return *this;
}

const CovarianceMatrix& GlobalUpdateMeasurement::covariance() const { return covariance_; }
CovarianceMatrix& GlobalUpdateMeasurement::covariance() { return covariance_; }
const Sophus::SO3d& GlobalUpdateMeasurement::attitude() const { return attitude_; }
Sophus::SO3d& GlobalUpdateMeasurement::attitude() { return attitude_; }
const Eigen::Vector3d& GlobalUpdateMeasurement::position() const { return position_; }
Eigen::Vector3d& GlobalUpdateMeasurement::position() { return position_; }
uint64_t GlobalUpdateMeasurement::timeUSec() const { return time_usec_; }
void GlobalUpdateMeasurement::setTime(uint64_t time_usec) { time_usec_ = time_usec; }
std::ostream& operator<<(std::ostream& os, const GlobalUpdateMeasurement& meas)
{
    std::cout << "Global - Time: " << meas.timeUSec() << '\n';
    const Eigen::Quaterniond& q = meas.attitude().unit_quaternion();
    std::cout << "Global - Attitude: " << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z()
              << '\n';
    std::cout << "Global - Position: " << meas.position().transpose() << '\n';
    return os;
}
}
}
}