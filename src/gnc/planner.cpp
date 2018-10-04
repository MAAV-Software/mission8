#include "gnc/planner.hpp"

namespace maav
{
namespace gnc
{
Planner::Planner(const std::string& config_path) : config_file{config_path} {}
void Planner::set_target() {}
void Planner::add_state(const State& state) {}
void Planner::add_map(/*const Map& map*/) {}
}  // namespace gnc
}  // namespace maav