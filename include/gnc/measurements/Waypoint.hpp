#pragma once

#include <Eigen/Eigen>

namespace maav
{
namespace gnc
{
struct Waypoint
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double yaw;
};
}
}