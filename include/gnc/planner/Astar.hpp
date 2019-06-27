#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <memory>
#include "gnc/State.hpp"
#include "gnc/planner/Path.hpp"

namespace maav
{
namespace gnc
{
namespace planner
{
/**
 * @brief A-star planning object 
 */
class Astar
{
public:
	/**
	 * @brief returns a path using A* Search on the given params
	 * @param start		starting point for path, Waypoint type
	 * @param goal	 	goal Waypoint 
	 * @param tree	    Octomap with obstacles
	 * @return          path from start to goal, 
	 					returns path containing only the start node if goal unreachable
	 */
	Path operator()(const Waypoint& start, const Waypoint& goal, const std::shared_ptr<octomap::OcTree> tree);
};

}
}
}

#endif /* ASTAR_HPP */
