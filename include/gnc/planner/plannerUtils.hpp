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
    size_t operator[](const point3d& a)
    {
        point3d map_origin(0.0, 0.0, a.z());
        if(map_origin == a)
        {
            return 0;
        }
        unsigned ringNum = l1norm(a, map_origin) / resolution_;
        unsigned ringSize = ringNum * 4;
        // arithmetic sum
        unsigned prevBlocks = ((8 + (ringNum - 1) * 4) * ((double)ringNum/2)) - ringSize;
        point3d corner;
        unsigned prevRingBlocks = 0;
        if( a.x() > 0 && a.y() >= 0) {
            corner = point3d(ringNum*resolution_, 0.0, a.z());
            prevRingBlocks = ringSize * 0.75;
        }   
        else if( a.x() <= 0 && a.y() > 0) {
            corner = point3d(0.0, ringNum*resolution_, a.z());
            prevRingBlocks = 0;
        }
        else if( a.x() < 0 && a.y() <= 0 ) {
            corner = point3d(-1.0 * ringNum*resolution_, 0.0, a.z());
            prevRingBlocks = ringSize * 0.25;
        }
        else {
            corner = point3d(0.0, -1.0 * ringNum*resolution_, a.z());
            prevRingBlocks = ringSize * 0.5;
        }
        return 1 + (prevBlocks + prevRingBlocks + l2norm(a, corner) / resolution_);
    }
    // TODO: Do the math for this
    size_t operator()(const point3d& a)
    {
        // Assume the arena size is 601 x 601 gridunits at 0.05 res. 301,301 is middle
        // scaline
        // X is row, Y is col
        point3d scaled = (a * (1. / resolution_));
        scaled += point3d(301.0, 301.0, 0.0);
        return (size_t)(scaled.x()*601.0 + scaled.y());
    }
    double resolution_;
};

}
}
}
#endif  // plannerUtils.hpp