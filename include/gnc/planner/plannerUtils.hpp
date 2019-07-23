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

// gets the linear index from a 2D grid. Counts N/W/S/E
// TODO: Fix the special way
struct GetId
{
    GetId(double resolution) : resolution_(resolution) {}
    // return linear index from the 2d grid
    size_t operator()(const point3d& a)
    {
        // Assume the arena size is 601 x 601 gridunits at 0.05 res. 301,301 is middle
        // scaline
        // X is row, Y is col
        point3d scaled = (a * (1. / resolution_));
        scaled += point3d(301.0, 301.0, 0.0);
        assert(scaled.x() > 0);
        assert(scaled.y() > 0);
        return std::round((scaled.x()*601.0 + scaled.y()));
    }
    double resolution_;
};

}
}
}
#endif  // plannerUtils.hpp