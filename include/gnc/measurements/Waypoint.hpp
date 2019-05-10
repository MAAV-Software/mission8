#pragma once

#include <Eigen/Eigen>
#include <iostream>

namespace maav
{
namespace gnc
{
struct Waypoint
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double yaw;
    double yaw_rate = 0;
    Waypoint() = default;
    Waypoint(Eigen::Vector3d pos, Eigen::Vector3d vel, double yaw_in) 
    	: position(pos), velocity(vel), yaw(yaw_in) {}

    // TODO: update this to include other values
    bool operator==(const Waypoint& other) const {
    	return position == other.position;
    }

    // Needed for boost test cases
    friend std::ostream& operator<<(std::ostream &out, const Waypoint& other) {
    	out << other.position;
    	return out;
    }
};
}
}