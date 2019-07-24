#ifndef PLANNER_UTILS_HPP
#define PLANNER_UTILS_HPP

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace maav
{
namespace gnc
{
namespace planner
{

using octomap::point3d;

double l1norm(const point3d& a, const point3d& b)
{
    point3d c = a - b;
    return abs(c.x()) + abs(c.y()) + abs(c.z());
}

double l2norm(const point3d& a, const point3d& b)
{
    point3d c = a - b;
    return c.norm();
}

struct GetId
{
    GetId(double resolution) : resolution_(resolution)
        {
            size_t units = std::round((double) MAX_ARENA_SIZE / resolution_);
            if (units % 2 == 0) { units += 1; }
            arena_grid_row_units_ = units;
            size_t midpt = ceil((double) units / 2.0);
            translation_ = point3d(midpt, midpt, 0.0);
        }
    // return linear index from the 2d grid
    size_t operator()(const point3d& a)
    {
        // X is row, Y is col
        point3d scaled = (a * (1. / resolution_));
        scaled += translation_;
        assert(scaled.x() > 0);
        assert(scaled.y() > 0);
        return std::round((scaled.x()*arena_grid_row_units_ + scaled.y()));
    }
    double resolution_;
    point3d translation_;
    size_t arena_grid_row_units_;
    const unsigned int MAX_ARENA_SIZE = 60; // in meters
};

}
}
}
#endif  // plannerUtils.hpp