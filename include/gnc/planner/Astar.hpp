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
	 * @param start		starting point for path, State type
	 * @param goal	 	goal Waypoint 
	 * @param map	    grid map with obstacles
	 * @return          path from start to goal, 
	 					returns path containing only the start node if goal unreachable
	 */
	Path operator()(const Waypoint& start, const Waypoint& goal, const std::shared_ptr<octomap::OcTree> tree);

private:
	/**
	 * @brief return distance between nodes in the map.
	 * @param a		starting point for path, State type
	 * @param b 	goal state for the
	 * @param map	grid map with obstacles
	 * @param l1	true for l1 norm, false for l2 norm (euclidean distance)
	 */
	double dist(const octomap::OcTreeNode* a, const octomap::OcTreeNode* b, 
		const std::shared_ptr<octomap::OcTree> tree, bool l1 = false);
};

}
}
}

#endif /* ASTAR_HPP */
