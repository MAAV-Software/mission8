#pragma once

#include <vector>
#include <gnc/measurements/Waypoint.hpp>

namespace maav
{
namespace gnc
{
    struct Path{
        int64_t utime;                      // Time of path creation

        std::vector<Waypoint> waypoints;

        Waypoint& operator[](int index)
        {
        	return waypoints[index];
        }

        bool operator==(const Path& other) const;
    };
}
}