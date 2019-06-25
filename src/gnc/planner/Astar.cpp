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

//TODO: remove after debugging
using std::cerr;
using std::cout;
using std::endl;

using namespace octomap;

namespace maav
{
namespace gnc
{
namespace planner
{

const unsigned int ARENA_SIZE = 30; // in meters 

double l1norm(const point3d& a, const point3d& b);
double l2norm(const point3d& a, const point3d& b);

// static double pnorm(const Eigen::VectorXd& v, bool l1 = false);
bool isCollision(const point3d& query, const OcTree* tree)
{
    // probability that cell is obstacle is less than 67%
    //return map.cellOdds(x, y) > 2.0;
    OcTreeNode* result = tree->search(query);
    if(!result) { return false; } // TODO: This means it was unknown. Handle later
    return result->getOccupancy() > 0.5;

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

// gets the linear index from a 2D grid. Counts N/W/S/E
struct GetId
{
    GetId(double resolution) : resolution_(resolution) {}
    // return linear index from the 2d grid
    size_t operator()(const point3d& a)
    {
        point3d map_origin(0, 0, a.z());
        if(map_origin == a)
        {
            return 0;
        }
        unsigned ringNum = l1norm(a, map_origin) / resolution_;
        unsigned ringSize = ringNum * 4;
        // arithmetic sum
        unsigned prevBlocks = ((8 + (ringNum - 1) * 4) * ((double)ringNum/2)) - ringSize;
        point3d corner;
        unsigned prevRingBlocks = 0;
        if( a.x() > 0 && a.y() >= 0) {
            corner = point3d(ringNum, 0, a.z());
            prevRingBlocks = ringSize * 0.75;
        }   
        else if( a.x() <= 0 && a.y() > 0) {
            corner = point3d(0.0, ringNum, a.z());
            prevRingBlocks = 0;
        }
        else if( a.x() < 0 && a.y() <= 0 ) {
            corner = point3d(-1.0 * ringNum, 0.0, a.z());
            prevRingBlocks = ringSize * 0.25;
        }
        else {
            corner = point3d(0.0, -1.0 * ringNum, a.z());
            prevRingBlocks = ringSize * 0.5;
        }
        return 1 + (prevBlocks + prevRingBlocks + l2norm(a, corner) / resolution_);
    }
    double resolution_;
};

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

    cout << "start: " << start_coord << "\n";
    cout << "goal: " << goal_coord << "\n";

    GetId getId(tree->getResolution());

    const size_t start_id = getId(start_coord);
    const size_t goal_id = getId(goal_coord);
    // keep track of visited nodes based on their id
    unordered_map<int, shared_ptr<Node> > visitedNodes;
    // construct priority queue on nodes and add start node
    // top element has least cost
    set<shared_ptr<Node> >  openNodes;
    openNodes.emplace(new Node(start_key, start_id, start_id, 0,
        l2norm(goal_coord, start_coord)));

    double stepSize = pow(2.0, level); // size to search the next key
    //double searchTolerance = tree->getResolution() * 10;
    // TODO: Add faults
    bool foundGoal = false;
    // algorithmic step
    auto searchStep = [&](const OcTreeKey& currKey, shared_ptr<Node> parent) {
        // check for have we visited the node 
        point3d currCoord = tree->keyToCoord(currKey, depth);
        size_t currId = getId(currCoord);
        auto itClosed = visitedNodes.find(currId);
        if(itClosed != visitedNodes.end()) { return; }
        // Check for collisions on node
        if(!isCollision(currCoord, tree.get()))
        {
            shared_ptr<Node> new_ptr = shared_ptr<Node>(new
                Node(currKey, currId, parent->id(),
                parent->getPathCost() + (tree->keyToCoord(parent->key(), depth) - currCoord).norm(),
                l2norm(currCoord, goal_coord))); // TODO: add support for pnorm

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
            visitedNodes[currId] = shared_ptr<Node>(new Node());
        }
    };
    unsigned long counter = 0;
    while (!openNodes.empty())
    {
        // assert cost of the beginning node is less than the last node
        assert(*openNodes.begin() < *openNodes.rbegin() ||  
            openNodes.begin()->cost() == openNodes.rbegin()->cost());
        shared_ptr<Node> n = *openNodes.begin();
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
        // Move up and down
        searchStep(OcTreeKey(curr_key[0], curr_key[1], 
            curr_key[2] + stepSize), n);
        searchStep(OcTreeKey(curr_key[0], curr_key[1], 
            curr_key[2] - stepSize), n);

        counter++;
        // Helpful debugging info to ensure the first moves are correct
        if(counter < 5)
        {
            cerr << "round done " << counter << "\n";
            n->printNode();
            cerr << tree->keyToCoord(curr_key, depth) << std::endl;
        }
    }
    cout << "took " << counter << " iterations until path found\n";
    if(!foundGoal) {
        // TODO: Add handling for not found goal
        assert(false);
    }
    // backtrack the found path
    vector<shared_ptr<Node> > node_path;
    shared_ptr<Node> current_node = visitedNodes[goal_id];
    while(current_node->id() != start_id){
        node_path.push_back(current_node);
        current_node = visitedNodes[current_node->parent()];
    }
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
            // waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),
            //     rad_to_deg(yaw_between(pos, start.position)));
            waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),0);
        }
        else
        {
            // TODO: Calculate Rates when controller has implemented it
            // waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),
            //     rad_to_deg(yaw_between(pos, waypoints.back().position)));
            waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),0);
            
        }
    }
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

double l2norm(const point3d& a, const point3d& b)
{
    point3d c = a - b;
    return c.norm();
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
