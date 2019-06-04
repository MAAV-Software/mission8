#include <vector>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include "common/math/math.hpp"
#include "gnc/planner/Astar.hpp"
#include "gnc/planner/Node.hpp"

using std::vector;
using std::priority_queue;
using std::unordered_map;
using std::cerr;

using namespace octomap;

namespace maav
{
namespace gnc
{
namespace planner
{

static double pnorm(const Eigen::VectorXd& v, bool l1 = false);

bool isCollision(const point3d& query, const OcTree* tree)
{
	// probability that cell is obstacle is less than 67%
	//return map.cellOdds(x, y) > 2.0;
	OcTreeNode* result = tree->search(query);
	if(!result) { return false; } // TODO: This means it was unknown. Handle later
	return result->getOccupancy() > 0.67;

}

Path Astar::operator()(const Waypoint& start, const Waypoint& goal, const std::shared_ptr<octomap::OcTree> tree)
{
	// TODO: GET MAP ORIGIN FROM DZ
	// adjust start and goal coordinates with the map's origin in the world frame
	//const point3d map_origin  =  map.originInGlobalFrame().cast<double>();
	const point3d map_origin(0,0,0);
	point3d start_coord(start.position.x(), start.position.y(), start.position.z());
	point3d goal_coord(goal.position.x(), goal.position.y(), goal.position.z());
	start_coord -= map_origin;
	goal_coord  -= map_origin;
	
	const OcTreeKey start_key = tree->coordToKey(start_coord); 
	const OcTreeKey goal_key = tree->coordToKey(goal_coord); 
	const auto getId = OcTreeKey::KeyHash();
	const int start_id = (int) getId(start_key);
	const int goal_id = (int) getId(goal_key);
	
	// construct priority queue on nodes and add start node
	// top element has least cost
	priority_queue<Node, vector<Node>, std::greater<Node> > openNodes;
	openNodes.push({start_key, start_id, -1, 0, 
		(goal_coord - start_coord).norm()});
	
	// keep track of visited nodes based on their id
	unordered_map<int, Node> visitedNodes;
	while (!openNodes.empty())
	{
		auto n = openNodes.top();
		openNodes.pop();

		// check if we've already visited the node
		if (visitedNodes.find(n.id()) != visitedNodes.end())
		{
			continue;
		}

		visitedNodes[n.id()] = n;

		// check if node is the goal node
		if (n.id() == goal_id)
		{
			break;
		}

		point3d n_coord = tree->keyToCoord(n.key());
		double x = n_coord.x(), y = n_coord.y(), z = n_coord.z();

		// test 8 moves in 2d plane
		for (int j = y - 1; j < y + 2; ++j) 
		{
			for(int i = x - 1; i < x + 2; ++i) 
			{
				// Is node the same as parent
				if(i == x && j == y) {
					continue;
				}
 
				// Check for collisions on node
				OcTreeKey currKey;
				point3d curr_coord(i, j, z);
				if(tree->coordToKeyChecked(curr_coord, currKey) && 
					!isCollision(curr_coord, tree.get()))
				{
					openNodes.push({currKey, (int) getId(currKey), n.id(), 
						(curr_coord - start_coord).norm(), 
						(curr_coord - goal_coord).norm() });
				}
			}
		}

		// test move up or down
		for (int k = z - 1; k < z + 2; k += 2)
		{
			OcTreeKey currKey;
			point3d curr_coord(x, y, k);
			if(tree->coordToKeyChecked(curr_coord, currKey) && 
				!isCollision(curr_coord, tree.get()))
			{
				openNodes.push({currKey, (int) getId(currKey), n.id(), 
					(curr_coord - start_coord).norm(), 
					(curr_coord - goal_coord).norm() });
			}
		}
	}

	vector<Node> node_path;
	auto current_node = visitedNodes[goal_id];
	while(current_node.parent() != -1){
		node_path.emplace_back(current_node);
		current_node = visitedNodes[current_node.parent()];
	}
	vector<Waypoint> waypoints;
	for(size_t i = node_path.size() - 2; i > 0; --i)
	{
		auto n = node_path[i];
		// translates to globalFrame
		auto tmp_pos = tree->keyToCoord(n.key());
		tmp_pos += map_origin;
		Eigen::Vector3d pos = Eigen::Vector3d(tmp_pos.x(), tmp_pos.y(), 
			tmp_pos.z());
		if(i == node_path.size() - 2)
		{
			// handles first waypoint past the start
			// TODO: Calculate Rates when controller has implemented it
			waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0), 
				yaw_between(pos, start.position));
		}
		else 
		{
			// TODO: Calculate Rates when controller has implemented it
			waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0), 
				yaw_between(pos, waypoints.back().position));
		}
	}

	Path path;
	path.waypoints = waypoints;
	path.utime = std::chrono::duration_cast<std::chrono::microseconds>
				 (std::chrono::system_clock::now().time_since_epoch()).count();
	return path;
}

double pnorm(const Eigen::VectorXd& v, bool l1)
{
	return l1 ? v.lpNorm<1>() : v.norm();
}

double Astar::dist(const octomap::OcTreeNode* a, const octomap::OcTreeNode* b, 
	const std::shared_ptr<octomap::OcTree> tree, bool l1)

{
	// TODO: Let the A* search use something other than L2
	assert(false);
	return 0.0;
	// const auto d = map.lin2coord(a.id()) - map.lin2coord(b.id());
	const auto d = Eigen::Vector3d(0,0,0);
	return pnorm(d, l1);
}

} // close planner namespace
} // close gnc namespace
} // close maav namespace
