#include <vector>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <memory>
#include "common/math/math.hpp"
#include "gnc/planner/Astar.hpp"
#include "gnc/planner/Node.hpp"

using std::vector;
using std::shared_ptr;
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

// static double pnorm(const Eigen::VectorXd& v, bool l1 = false);

bool isCollision(const point3d& query, const OcTree* tree)
{
    // probability that cell is obstacle is less than 67%
    //return map.cellOdds(x, y) > 2.0;
    OcTreeNode* result = tree->search(query);
    if(!result) { return false; } // TODO: This means it was unknown. Handle later
    return result->getOccupancy() > 0.67;

}

bool operator>(shared_ptr<Node> lhs, shared_ptr<Node> rhs)
{
    return *lhs > *rhs;
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
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node> >,
        std::greater<shared_ptr<Node> > > openNodes;
    openNodes.emplace(new Node(start_key, start_id, -1, 0,
        (goal_coord - start_coord).norm()));

    // keep track of visited nodes based on their id
    unordered_map<int, shared_ptr<Node> > visitedNodes;
    // TODO: Add faults
    bool foundGoal = false;
    try {
    while (!openNodes.empty())
    {
        auto n = openNodes.top();
        openNodes.pop();

        // check if we've already visited the node
        if (visitedNodes.find(n->id()) != visitedNodes.end())
        {
            continue;
        }

        visitedNodes[n->id()] = n;

        // check if node is the goal node
        if (n->id() == goal_id)
        {
            foundGoal = true;
            std::cout << "Found goal" << foundGoal << std::endl;
            break;
        }

        point3d n_coord = tree->keyToCoord(n->key());
        double x = n_coord.x(), y = n_coord.y(), z = n_coord.z();

        double tolerance = tree->getResolution() * 3;
        // test 8 moves in 2d plane
        for (int j = y - tolerance; j < y + 2*tolerance; j += tolerance)
        {
            for(int i = x - tolerance; i < x + 2*tolerance; i += tolerance)
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
                    shared_ptr<Node> new_ptr = shared_ptr<Node>(new
                        Node(currKey, (int) getId(currKey), n->id(),
                        (curr_coord - start_coord).norm(),
                        (curr_coord - goal_coord).norm()));
                    // check for the coordinate in the correct tolerance to be
                    // considered finding our goal
                    if ((curr_coord - goal_coord).norm() < tolerance)
                    {
                        new_ptr->setId(goal_id);
                        visitedNodes[goal_id] = new_ptr;
                        std::cout << "found goal" << std::endl;
                        foundGoal = true;
                        throw 15;
                    }

                    openNodes.push(new_ptr);
                }
            }
        }

        // test move up or down
        /*
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
        */
    }}
    catch(int e) {}
    std::cout << "Computed path." << std::endl;

    vector<shared_ptr<Node> > node_path;
    shared_ptr<Node> current_node = visitedNodes[goal_id];
    std::cout << "Got current_node from goal id" << std::endl;
    while(current_node->parent() != -1){
        node_path.push_back(current_node);
        std::cout << "Current node pushed back" << std::endl;
        std::cout << current_node->parent() << std::endl;
        current_node = visitedNodes[current_node->parent()];
        std::cout << "Got current_node from parent." << std::endl;
    }
    std::cout << "Path is in list now." << std::endl;
    vector<Waypoint> waypoints;
    for(int i = node_path.size() - 2; i > 0; --i)
    {
        shared_ptr<Node> n = node_path[i];
        // translates to globalFrame
        auto tmp_pos = tree->keyToCoord(n->key());
        tmp_pos += map_origin;
        Eigen::Vector3d pos = Eigen::Vector3d(tmp_pos.x(), tmp_pos.y(),
            tmp_pos.z());
        if(i == (int)node_path.size() - 2)
        {
            // handles first waypoint past the start
            // TODO: Calculate Rates when controller has implemented it
            waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),
                rad_to_deg(yaw_between(pos, start.position)));
        }
        else
        {
            // TODO: Calculate Rates when controller has implemented it
            waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),
                rad_to_deg(yaw_between(pos, waypoints.back().position)));
        }
    }
    std::cout << "Path object created." << std::endl;

    Path path;
    path.waypoints = waypoints;
    path.utime = std::chrono::duration_cast<std::chrono::microseconds>
                 (std::chrono::system_clock::now().time_since_epoch()).count();
    return path;
}

/*
double pnorm(const Eigen::VectorXd& v, bool l1)
{
    return l1 ? v.lpNorm<1>() : v.norm();
}
*/

/*
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
*/

} // close planner namespace
} // close gnc namespace
} // close maav namespace
