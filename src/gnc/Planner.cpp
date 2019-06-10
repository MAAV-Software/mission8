#include "gnc/Planner.hpp"
#include <iostream>

namespace maav
{
namespace gnc
{
Planner::Planner(const std::string& config_path)
	: config_file(config_path) {}

Path Planner::get_path() {
	if(!tree_) { return Path(); }
	return astar(state_, target_, tree_);
}

void Planner::print_path(Path& path)
{
    std::cout << "{ " << std::endl;
    for(auto& w: path.waypoints)
    {
        std::cout << "[ ";
        std::cout << w.position[0] << ", ";
        std::cout << w.position[1] << ", ";
        std::cout << w.position[2];
        std::cout << " ]" << std::endl;
    }
    std::cout << " }" << std::endl;
}

void Planner::update_target(const Waypoint& target) { target_ = target; }
void Planner::update_state(const State& state) {
	state_.position = state.position();
	state_.velocity = state.velocity();
	state_.yaw =  state.attitude().angleZ();
}
void Planner::update_map(const std::shared_ptr<octomap::OcTree> tree) {
	tree_ = tree;
}
}  // namespace gnc
}  // namespace maav
