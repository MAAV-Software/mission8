#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <gnc/State.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "gnc/planner/Astar.hpp"
#include "gnc/planner/Path.hpp"

namespace maav
{
namespace gnc
{
class Planner
{
public:
    Planner(const std::string& path_config);

    // returns the path
    Path get_path();

    void update_target(const Waypoint& target);

    void update_state(const State& state);

    void update_map(const std::shared_ptr<octomap::OcTree> tree);

private:
    std::string config_file;
    planner::Astar astar;
    std::shared_ptr<octomap::OcTree> tree_ = nullptr;
    Waypoint target_;
    Waypoint state_;
};

}  // namespace gnc
}  // namespace maav

#endif /* PLANNER_HPP */