#include <algorithm>
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
#include "gnc/planner/plannerUtils.hpp"

using std::vector;
using std::shared_ptr;
using std::set;
using std::unordered_map;
using std::make_shared;
using std::find_if;

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

Astar::Astar(const YAML::Node& config) : 
    min_obstacle_dist_(config["min_dist_to_obstacle"].as<double>()), 
    occupancy_thresh_(config["occupancy_thresh"].as<double>()),
    tree_level_(config["tree_resolution_level"].as<unsigned int>()) {}
/*
* A point has no collision if it is a safe distance away from the nearest
* obstacle.
* TODO: Support for moving obstacles
*/
bool Astar::isCollision(const point3d& query, const OcTree* tree)
{
    OcTreeNode* result = tree->search(query);
    if(result && result->getOccupancy() > occupancy_thresh_)
        return true;
    
    // Cast a ray in 8 directions and see if there is any obstacle within 0.5m
    point3d hitPt;
    if(tree->castRay(query, point3d(0.0, 1.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(1.0, 0.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(0.0, -1.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(-1.0, 0.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(1.0, 1.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(-1.0, 1.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(1.0, -1.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    else if(tree->castRay(query, point3d(-1.0, -1.0, 0.0), hitPt, true, min_obstacle_dist_)) {
        return true;
    }
    return false;
}

Path Astar::operator()(const Waypoint& start, const Waypoint& goal, const std::shared_ptr<octomap::OcTree> tree)
{   
    // TODO: GET MAP ORIGIN
    // adjust start and goal coordinates with the map's origin in the world frame
    //const point3d map_origin  =  map.originInGlobalFrame().cast<double>();
    const point3d map_origin(0.0, 0.0, 0.0);
    point3d start_coord(start.position.x(), start.position.y(), start.position.z());
    point3d goal_coord(goal.position.x(), goal.position.y(), goal.position.z());
    start_coord -= map_origin;
    goal_coord  -= map_origin;

    // keys are used to iterate through voxels
    // a higher depth is smaller resolution 
    // key[0] + 1 is equivalent to coord[0] + res
    unsigned max_depth = 16; // TODO: check that this is true
    unsigned depth = max_depth - tree_level_;
    const OcTreeKey start_key = tree->coordToKey(start_coord, depth);
    const OcTreeKey goal_key = tree->coordToKey(goal_coord, depth);

    // update coordinates to be in the same depth
    start_coord = tree->keyToCoord(start_key, depth);
    goal_coord = tree->keyToCoord(goal_key, depth);

    cout << "start: " << start_coord << "\n";
    cout << "goal: " << goal_coord << "\n";

    GetId getId(tree->getResolution());
    //auto getId = OcTreeKey::KeyHash();

    const size_t start_id = getId(start_coord);
    const size_t goal_id = getId(goal_coord);
    // keep track of visited nodes based on their id
    unordered_map<size_t, shared_ptr<Node> > visitedNodes;
    // construct priority queue on nodes and add start node
    // top element has least cost
    auto comp = [](const auto &a, const auto &b) { return *a < *b; };
    set<shared_ptr<Node>, decltype(comp) >  openNodes(comp);
    openNodes.emplace(new Node(start_key, start_id, start_id, 0,
        l2norm(goal_coord, start_coord)));

    double stepSize = pow(2.0, tree_level_); // size to search the next key
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
            shared_ptr<Node> new_ptr = make_shared<Node>(currKey, currId, 
                parent->id(), parent->getPathCost() + 
                l2norm(tree->keyToCoord(parent->key(), depth), currCoord),
                l2norm(currCoord, goal_coord));

            auto itOpen = find_if(openNodes.cbegin(), openNodes.cend(), 
                [&new_ptr](const auto &b) {return *new_ptr == *b;} );    
            if(itOpen != openNodes.end() &&
                new_ptr->getPathCost() >= (*itOpen)->getPathCost()) 
            {
                return; 
            }
            else if(itOpen != openNodes.end() &&
                new_ptr->getPathCost() < (*itOpen)->getPathCost())
            {
                openNodes.erase(itOpen);
                openNodes.insert(new_ptr);
                return;
            }
            openNodes.insert(new_ptr);
        }
        else 
        {
            // store collision as visited already so we never revisit
            visitedNodes[currId] = make_shared<Node>();
        }
    };
    unsigned long counter = 0;
    while (!openNodes.empty())
    {
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

        // Add diagnols
       searchStep(OcTreeKey(curr_key[0] + stepSize, curr_key[1] + stepSize, 
           curr_key[2]), n);
       searchStep(OcTreeKey(curr_key[0] - stepSize, curr_key[1] + stepSize, 
           curr_key[2]), n);
       searchStep(OcTreeKey(curr_key[0] + stepSize, curr_key[1] - stepSize, 
           curr_key[2]), n);
       searchStep(OcTreeKey(curr_key[0] - stepSize, curr_key[1] - stepSize, 
           curr_key[2]), n);

        // Move up and down
        // searchStep(OcTreeKey(curr_key[0], curr_key[1], 
        //     curr_key[2] + stepSize), n);
        // searchStep(OcTreeKey(curr_key[0], curr_key[1], 
        //     curr_key[2] - stepSize), n);

        counter++;
        // Helpful debugging info to ensure the first moves are correct
        // if(counter < 5)
        // {
        //     cerr << "round done " << counter << "\n";
        //     n->printNode();
        //     cerr << tree->keyToCoord(curr_key, depth) << std::endl;
        // }
    }
    cout << "took " << counter << " iterations until path found\n";
    if(!foundGoal) {
        // Returns a path with only the starting waypoint
        cout << "Did not find a goal" << endl;
        Path path;
        path.waypoints.push_back(start);
        path.utime = std::chrono::duration_cast<std::chrono::microseconds>
                     (std::chrono::system_clock::now().time_since_epoch()).count();
        return path;
    }
    // backtrack the found path. node_path[0] is the goal
    vector<shared_ptr<Node> > node_path;
    shared_ptr<Node> current_node = visitedNodes[goal_id];
    while(current_node->id() != start_id){
        node_path.push_back(current_node);
        current_node = visitedNodes[current_node->parent()];
    }
    vector<Waypoint> waypoints;
    for(int i = node_path.size() - 1; i > 0; --i)
    {
        shared_ptr<Node> n = node_path[i];
        // translates to globalFrame
        auto tmp_pos = tree->keyToCoord(n->key(), depth);
        tmp_pos += map_origin;
        Eigen::Vector3d pos = Eigen::Vector3d(tmp_pos.x(), tmp_pos.y(),
            tmp_pos.z());
        if(i == (int)node_path.size() - 1)
        {
            // handles start waypoint
            waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0), yaw_between(start.position, pos));
        }
        else
        {
            waypoints.emplace_back(pos, Eigen::Vector3d(0,0,0),
                yaw_between(waypoints.back().position, pos));
            
        }
    }

    Path path;
    path.waypoints = newWaypoints;
    path.utime = std::chrono::duration_cast<std::chrono::microseconds>
                 (std::chrono::system_clock::now().time_since_epoch()).count();

    return path;
}

} // close planner namespace
} // close gnc namespace
} // close maav namespace
