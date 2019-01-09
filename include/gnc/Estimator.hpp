#pragma once

#include <yaml-cpp/yaml.h>

#include <gnc/State.hpp>
#include <gnc/kalman/History.hpp>
#include <gnc/kalman/Prediction.hpp>
#include <gnc/kalman/updates/GlobalUpdate.hpp>
#include <gnc/kalman/updates/LidarUpdate.hpp>
#include <gnc/kalman/updates/PlanefitUpdate.hpp>
#include <gnc/measurements/Measurement.hpp>

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

    void setBiases(const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias);

private:
    State empty_state_;
    kalman::History history_;
    kalman::UkfPrediction prediction_;
    kalman::LidarUpdate lidar_update_;
    kalman::PlaneFitUpdate planefit_update_;
    kalman::GlobalUpdate global_update_;
};

}  // namespace gnc
}  // namespace maav
