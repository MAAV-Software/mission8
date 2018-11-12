#pragma once

#include <yaml-cpp/yaml.h>

#include <gnc/kalman/history.hpp>
#include <gnc/kalman/prediction.hpp>
#include <gnc/kalman/updates/lidar_update.hpp>
#include <gnc/measurements/Measurement.hpp>
#include <gnc/state.hpp>

namespace maav
{
namespace gnc
{
class Estimator
{
    public:
    /**
     * @param config This yaml node requires at leas 4 keys: 'history', 'state', 'prediction',
     * and 'updates'
     */
    Estimator(YAML::Node config);

    /**
     * TODO: Update style
     * @bried Runs the kalman filter on a new set of measurements
     * @param meas A set of measurements. Imu must be populated
     */
    const State& add_measurement_set(const measurements::MeasurementSet& meas);

    private:
    State empty_state_;
    kalman::History history_;
    kalman::UkfPrediction prediction_;
    kalman::LidarUpdate lidar_update_;
};

}  // namespace gnc
}  // namespace maav
