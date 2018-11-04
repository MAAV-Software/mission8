#pragma once

#include <gnc/kalman/history.hpp>
#include <gnc/kalman/unscented_transform.hpp>
#include <gnc/measurements/Measurement.hpp>

using std::placeholders::_1;

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * TargetSpace is some space where we compare our prediction ot our sensor measurements.
 * Note that TargetSpace doesn't have to be a sensor measured.
 */
template <class TargetSpace>
class BaseUpdate
{
    private:
    using UT = UnscentedTransform<TargetSpace>;
    constexpr static size_t TargetDoF = TargetSpace::DoF;
    using CovarianceMatrix = typename TargetSpace::CovarianceMatrix;
    using ErrorStateVector = typename TargetSpace::ErrorStateVector;
    using CrossCovarianceMatrix = Eigen::Matrix<double, KalmanState::DoF, TargetDoF>;
    using KalmanGainMatrix = CrossCovarianceMatrix;

    public:
    BaseUpdate()
        : _unscented_transform(std::bind(&BaseUpdate::predicted, this, _1), 0.1, 2, 0.0),
          R(CovarianceMatrix::Zero())  // TODO: Read in from yaml
    {
    }
    /**
     * Nonlinear h function.
     * Maps the current state to some element of the target space
     */
    virtual TargetSpace predicted(const KalmanState& state) = 0;

    /**
     * Maps the actual sensor measurements to some element of the target space.
     */
    virtual TargetSpace measured(const measurements::Measurement& meas) = 0;

    /**
     * Updates the state based on the measurements in a snapshot.
     */
    virtual void operator()(History::Snapshot& snapshot)
    {
        KalmanState& state = snapshot.state;

        const TargetSpace predicted_meas = _unscented_transform(state);

        // Extract necessary variables from the UT
        const typename UT::Weights& c_weights = _unscented_transform.c_weights();
        const typename UT::SigmaPoints& sigma_points = _unscented_transform.last_sigma_points();
        const typename UT::TransformedPoints& transformed_points =
            _unscented_transform.last_transformed_points();

        const CovarianceMatrix S = predicted_meas.covariance() + R;
        CrossCovarianceMatrix Sigma_x_z = CrossCovarianceMatrix::Zero();
        for (size_t i = 0; i < UnscentedTransform<TargetSpace>::N; i++)
        {
            Sigma_x_z +=
                c_weights[i] * (sigma_points[i] - state) * (transformed_points[i] - predicted_meas);
        }
        const KalmanGainMatrix K = Sigma_x_z * S.inverse();
        ErrorStateVector residual = measured(snapshot.measurement) - predicted_meas;

        // Update the state gaussian in place
        snapshot.state += K * residual;
        snapshot.state.covariance() -= K * S * K.transpose();
    }

    private:
    UT _unscented_transform;
    CovarianceMatrix R;
};
}
}
}
