#pragma once

#include <gnc/control/ContinuousPath.hpp>

#include <map>

namespace maav
{
namespace gnc
{
/**
 * An implementation of a continuous path that interpolates between waypoints linearly
 */
class LinearlyInterpolatedPath : public ContinuousPath
{
public:
    LinearlyInterpolatedPath(const path_t& path, double speed);

    virtual void updatePath(const path_t& path) override;
    virtual Waypoint sample(uint64_t time) const override;

protected:
    double speed_;
    std::map<double, size_t> waypoint_dictionary_;
};
}  // namespace gnc
}  // namespace maav