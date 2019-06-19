#include <vector>
#include <unordered_map>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <set>
#include <cmath>
#include <memory>
#include "common/math/math.hpp"
#include "gnc/planner/Astar.hpp"
#include "gnc/planner/Node.hpp"

using std::vector;
using std::shared_ptr;
using std::set;
using std::unordered_map;
using std::cerr;

using namespace octomap;

namespace maav
{
namespace gnc
{
namespace planner
{

double l1norm(const point3d& a, const point3d& b);

// static double pnorm(const Eigen::VectorXd& v, bool l1 = false);
bool isCollision(const point3d& query, const OcTree* tree)
{
    // probability that cell is obstacle is less than 67%
    //return map.cellOdds(x, y) > 2.0;
    OcTreeNode* result = tree->search(query);
    if(!result) { return false; } // TODO: This means it was unknown. Handle later
    return result->getOccupancy() > 0.67;

}

// operator overload for priority queue
bool operator>(shared_ptr<Node> lhs, shared_ptr<Node> rhs)
{
    return *lhs > *rhs;
}

// operator overload for priority queue
bool operator<(shared_ptr<Node> lhs, shared_ptr<Node> rhs)
{
    return *lhs < *rhs;
}

bool operator==(shared_ptr<Node> lhs, shared_ptr<Node> rhs)
{
    return *lhs == *rhs;
}


Path Astar::operator()(const Waypoint& start, const Waypoint& goal, const std::shared_ptr<octomap::OcTree> tree)
{
    cerr << "starting A*\n";
    // TODO: GET MAP ORIGIN FROM DZ
    // adjust start and goal coordinates with the map's origin in the world frame
    //const point3d map_origin  =  map.originInGlobalFrame().cast<double>();
    const point3d map_origin(0,0,0);
    point3d start_coord(start.position.x(), start.position.y(), start.position.z());
    point3d goal_coord(goal.position.x(), goal.position.y(), goal.position.z());
    start_coord -= map_origin;
    goal_coord  -= map_origin;

    // keys are used to iterate through voxels
    // a higher depth is smaller resolution 
    // key[0] + 1 is equivalent to coord[0] + res
    unsigned level = 2; // TODO: add to config
    unsigned max_depth = 16; // TODO: check that this is true
    unsigned depth = max_depth - level;
    const OcTreeKey start_key = tree->coordToKey(start_coord, depth);
    const OcTreeKey goal_key = tree->coordToKey(goal_coord, depth);

    // update coordinates to be in the same depth
    start_coord = tree->keyToCoord(start_key, depth);
    goal_coord = tree->keyToCoord(goal_key, depth);

    cerr << "start: " << start_coord << "\n";
    cerr << "goal: " << goal_coord << "\n";

    const auto getId = OcTreeKey::KeyHash(); // used for hashtable
    const size_t start_id = getId(start_key);
    const size_t goal_id = getId(goal_key);
    // keep track of visited nodes based on their id
    unordered_map<int, shared_ptr<Node> > visitedNodes;
    // construct priority queue on nodes and add start node
    // top element has least cost
    set<shared_ptr<Node> >  openNodes;
    openNodes.emplace(new Node(start_key, start_id, start_id, 0,
        l1norm(goal_coord, start_coord)));

    double stepSize = pow(2.0, level); // size to search the next key
    //double searchTolerance = tree->getResolution() * 10;
    // TODO: Add faults
    bool foundGoal = false;
    // algorithmic step
    auto searchStep = [&](const OcTreeKey& currKey, shared_ptr<Node> parent) {
        // check for have we visited the node 
        auto itClosed = visitedNodes.find( getId(currKey) );
        if(itClosed != visitedNodes.end()) { return; }
        // Check for collisions on node
        point3d currCoord = tree->keyToCoord(currKey, depth);
        if(!isCollision(currCoord, tree.get()))
        {
            shared_ptr<Node> new_ptr = shared_ptr<Node>(new
                Node(currKey, getId(currKey), parent->id(),
                parent->getPathCost() + (tree->keyToCoord(parent->key(), depth) - currCoord).norm(),
                l1norm(currCoord, goal_coord))); // TODO: add support for pnorm

            auto itOpen = openNodes.find(new_ptr);    
            if(itOpen != openNodes.end() &&
                new_ptr->getPathCost() > (*itOpen)->getPathCost()) 
            { 
                return; 
            }
            openNodes.insert(new_ptr);
        }
        else 
        {
            // store collision as visited already so we never revisit
            visitedNodes[getId(currKey)] = shared_ptr<Node>(new Node());
        }
    };
    unsigned long counter = 0;
    while (!openNodes.empty())
    {
        auto n = *openNodes.begin();
        openNodes.erase(openNodes.begin());
        // check for the coordinate in the correct tolerance to be
        // considered finding our goal
        if(n->id() == goal_id) {
            foundGoal = true;
            visitedNodes[n->id()] = n;
            break;
        }
        // check if we've already visited the node
        if (visitedNodes.find(n->id()) != visitedNodes.end())
        {
            continue;
        }
        // add that we have visited node
        visitedNodes[n->id()] = n;
        OcTreeKey curr_key = n->key();
        // Move forward, back, left, right
        searchStep(OcTreeKey(curr_key[0] + stepSize, curr_key[1], 
            curr_key[2]), n);
        searchStep(OcTreeKey(curr_key[0] - stepSize, curr_key[1], 
            curr_key[2]), n);
        searchStep(OcTreeKey(curr_key[0], curr_key[1] + stepSize, 
            curr_key[2]), n);
        searchStep(OcTreeKey(curr_key[0], curr_key[1] - stepSize, 
            curr_key[2]), n);
        searchStep(OcTreeKey(curr_key[0], curr_key[1], 
            curr_key[2] + stepSize), n);
        searchStep(OcTreeKey(curr_key[0], curr_key[1], 
            curr_key[2] - stepSize), n);
        

        counter++;
        if(counter < 5)
        {
            cerr << "round done " << counter << "\n";
            n->printNode();
            cerr << tree->keyToCoord(curr_key, depth) << std::endl;
            cerr << tree->keyToCoord(curr_key, depth) << std::endl;
        }
    }
    cerr << "took " << counter << " iterations until path found\n";
    if(!foundGoal) {
        // TODO: Add handling for not found goal
        assert(false);
    }
    // backtrack the found path
    vector<shared_ptr<Node> > node_path;
    shared_ptr<Node> current_node = visitedNodes[goal_id];
    std::cout << "Got current_node from goal id" << std::endl;
    while(current_node->id() != start_id){
        node_path.push_back(current_node);
        current_node = visitedNodes[current_node->parent()];
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

double l1norm(const point3d& a, const point3d& b)
{
    point3d c = a - b;
    return abs(c.x()) + abs(c.y()) + abs(c.z());
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
