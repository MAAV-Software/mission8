#pragma once

#include <yaml-cpp/yaml.h>

#include <common/utils/yaml_matrix.hpp>
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
    using CrossCovarianceMatrix = Eigen::Matrix<double, State::DoF, TargetDoF>;
    using KalmanGainMatrix = CrossCovarianceMatrix;

public:
    BaseUpdate(YAML::Node config)
        : unscented_transform_(config["UT"])
    {
        Eigen::Matrix<double, TargetDoF, 1> R_diag = config["R"].as<Eigen::Matrix<double, TargetDoF, 1>>();
        R_ = Eigen::DiagonalMatrix<double, TargetDoF>(R_diag);
        unscented_transform_.set_transformation(std::bind(&BaseUpdate::predicted, this, _1));
    }

protected:
    /**
     * Nonlinear h function.
     * Maps the current state to some element of the target space
     */
    virtual TargetSpace predicted(const State& state) = 0;

    /**
     * Maps the actual sensor measurements to some element of the target space.
     */
    virtual TargetSpace measured(const measurements::Measurement& meas) = 0;

    /**
     * Updates the state based on the measurements in a snapshot.
     */
    void correct(History::Snapshot& snapshot)
    {
        State& state = snapshot.state;

        const TargetSpace predicted_meas = unscented_transform_(state);

        // Extract necessary variables from the UT
        const typename UT::Weights& c_weights = unscented_transform_.c_weights();
        const typename UT::SigmaPoints& sigma_points = unscented_transform_.last_sigma_points();
        const typename UT::TransformedPoints& transformed_points =
            unscented_transform_.last_transformed_points();

        const CovarianceMatrix S = predicted_meas.covariance() + R_;
        CrossCovarianceMatrix Sigma_x_z = CrossCovarianceMatrix::Zero();
        for (size_t i = 0; i < UnscentedTransform<TargetSpace>::N; i++)
        {
            Sigma_x_z +=
                c_weights[i] * (sigma_points[i] - state) * (transformed_points[i] - predicted_meas).transpose();
        }
        const KalmanGainMatrix K = Sigma_x_z * S.inverse();
        ErrorStateVector residual = measured(snapshot.measurement) - predicted_meas;
        // Update the state gaussian in place
        state += K * residual;
        state.covariance() -= K * S * K.transpose();
    }

private:
    UT unscented_transform_;
    CovarianceMatrix R_;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace maav
