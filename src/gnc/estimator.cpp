#include <cmath>

#include <gnc/estimator.hpp>
#include <gnc/state/base_state.hpp>

using maav::gnc::measurements::MeasurementSet;

namespace maav
{
namespace gnc
{
using namespace kalman;

Estimator::Estimator(YAML::Node config)
    : empty_state_(0),
      history_(config["history"], config["state"]),
      prediction_(config["prediction"]),
      lidar_update_(config["updates"])
{
}

const State& Estimator::add_measurement_set(const MeasurementSet& meas)
{
    auto it_pair = history_.add_measurement(meas);
    History::Iterator begin = it_pair.first;
    History::Iterator end = it_pair.second;

    if (begin == end)
    {
        return empty_state_;
    }

    History::Iterator prev, next;
    next = prev = begin;
    next++;

    while (next != end)
    {
        prediction_(prev, next);
        lidar_update_(*next);

        // Limit covariance for now
        double cap = 0.1;
        KalmanState::CovarianceMatrix& cov = next->state.covariance();
        cov.block<2, KalmanState::DoF>(3, 0) = Eigen::Matrix<double, 2, KalmanState::DoF>::Zero();
        cov.block<2, KalmanState::DoF>(6, 0) = Eigen::Matrix<double, 2, KalmanState::DoF>::Zero();
        cov.block<KalmanState::DoF, 2>(0, 3) = Eigen::Matrix<double, KalmanState::DoF, 2>::Zero();
        cov.block<KalmanState::DoF, 2>(0, 6) = Eigen::Matrix<double, KalmanState::DoF, 2>::Zero();
        cov.block<2, 2>(3, 3) = cap * Eigen::Matrix2d::Identity();
        cov.block<2, 2>(6, 6) = cap * Eigen::Matrix2d::Identity();

        // TODO: Add updates
        prev++;
        next++;
    }

    std::cout << "Attitude: " << prev->state.attitude().unit_quaternion().w() << ' '
              << prev->state.attitude().unit_quaternion().x() << ' '
              << prev->state.attitude().unit_quaternion().y() << ' '
              << prev->state.attitude().unit_quaternion().z() << '\n';
    std::cout << "Position: " << prev->state.position().transpose() << '\n';
    std::cout << "Velocity: " << prev->state.velocity().transpose() << std::endl;
    std::cout << "Variance:\n" << prev->state.covariance().diagonal().transpose() << std::endl;
    return prev->state;
}

}  // namespace gnc
}  // namespace maav
