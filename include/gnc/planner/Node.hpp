#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace maav
{
namespace gnc
{
namespace planner
{

/**
 * @brief Node representation for A-star (or other sampling-based) planning
 */
class Node
{
public:
	/**
	 * @brief Default constructs Node with id and parent of -1
	 */
	Node() : id_{-1}, parent_{-1}, path_cost_{0}, heuristic_cost_{0} {}
	
	/**
	 * @brief Constructs a node with given parameters
	 * @param key 		key that is used to get coordinates from tree
	 * @param id		linear index into occupancy map for this node
	 * @param parent 	linear index into occupancy map for parent node (use -1
	 *					for the parent of the root node in the planning problem)
	 * @param path_cost	running path cost (usually L1 or L2 norm) from start/root
	 * @param heuristic_cost cost to goal node
	 */
	Node(octomap::OcTreeKey key, int id, int parent, double path_cost, double heuristic_cost) 
		: key_{key}, id_{id}, parent_{parent}, path_cost_{path_cost}, 
	  	  heuristic_cost_{heuristic_cost} {}
	
	/**
	 * @brief Less than comparison operator for this node
	 * @param rhs	Node object being compared to this object on the right-hand-side
	 *				of the < symbol (e.g., this_object < rhs)
	 */
	bool operator<(const Node& rhs) const { return cost() < rhs.cost(); }

	/**
	 * @brief Greater than comparison operator for this node
	 * @param rhs	Node object being compared to this object on the right-hand-side
	 *				of the < symbol (e.g., this_object < rhs)
	 */
	bool operator>(const Node& rhs) const { return cost() > rhs.cost(); }
	
	/**
	 * @brief Returns cost of this node used for planning
	 */
	double cost() const { return path_cost_ + heuristic_cost_; }
		
	/**
	 * @brief Returns node id (liner index into occupancy map)
	 */
	int id() const { return id_; }
	
	octomap::OcTreeKey key() const { return key_; }

	/**
	 * @brief Returns parent id (liner index into occupancy map)
	 */
	int parent() const { return parent_; }
	
private:
	octomap::OcTreeKey key_;
	int id_;					//< this node's linear index into occupancy map
	int parent_;				//< parent's linear index into occupancy map (-1 for root)
	double path_cost_;			//< cost from start node
	double heuristic_cost_;		//< heuristic cost to goal
};

} // close planner namespace
} // close gnc namespace
} // close maav namespace

#endif /* PLANNER_NODE_HPP */
