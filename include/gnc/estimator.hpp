#pragma once

#include <yaml-cpp/yaml.h>

#include <gnc/kalman/history.hpp>
#include <gnc/kalman/prediction.hpp>
#include <gnc/measurements/Measurement.hpp>
#include <gnc/state.hpp>

namespace maav
{
namespace gnc
{
class Estimator
{
    public:
    Estimator(YAML::Node config);

    const State& add_measurement_set(measurements::MeasurementSet& meas);

    private:
    State empty_state;

    kalman::History history;

    kalman::UkfPrediction prediction;
};

}  // namespace gnc
}  // namespace maav
