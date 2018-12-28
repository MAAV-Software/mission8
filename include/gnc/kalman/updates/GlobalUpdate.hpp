#ifndef GLOBAL_UPDATE_HPP
#define GLOBAL_UPDATE_HPP

#include <yaml-cpp/yaml.h>

#include <gnc/kalman/base_update.hpp>
#include <gnc/measurements/GlobalUpdate.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
/**
 * A correction step for a downward facing lidar on a flat surface
 */
class GlobalUpdate : public BaseUpdate<measurements::GlobalUpdateMeasurement>
{
public:
    /**
     *  @param config This yaml node must have a "lidar" key with unscented transform parameters and
     * a sensor covariance matrix, R
     */
    GlobalUpdate(YAML::Node config);

    /**
     * @breif Computes the observation model (h(x)) for a state provided by an UnscentedTransform
     * @param state A sigma point proved by an UnscentedTransform
     * @return The predicted lidar observation
     */
    measurements::GlobalUpdateMeasurement predicted(const State& state);

    /**
     * @param meas
     * @return The relevant lidar measurement from the list of measurements
     */
    measurements::GlobalUpdateMeasurement measured(const measurements::Measurement& meas);

    /**
     * @brief Performs the correction step for a lidar
     * @param snapshot A mutable reference to a point in time. The state will be updated according
     * to the sensor model
     */
    void operator()(History::Snapshot& snapshot);

private:
    using BaseUpdate<measurements::GlobalUpdateMeasurement>::correct;
};
}  // namespace kalman
}  // namespace gnc
}  // namespace maav

#endif
