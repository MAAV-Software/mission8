
#include <gnc/planner/motion_planner.hpp>
#include <gnc/planner/astar.hpp>
#include <gnc/planner/grid_utils.hpp>
#include <gnc/planner/path.hpp>
#include <gnc/measurements/Waypoint.hpp>
#include <chrono>

using Eigen::Vector2d;
using Eigen::Vector2i ;
using std::chrono::system_clock;

namespace maav
{
namespace gnc
{
MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


Path MotionPlanner::planPath(const Waypoint& start, 
                                     const Waypoint& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal) || !isValidStart(start))
    {
        Path failedPath;
        // TODO: Get utime to work
        // auto now = system_clock::now();
        // auto now_ms = std::chrono::time_point_cast<std::chrono::microseconds>(now);
        // auto value = std::chrono::duration_cast<std::chrono::microseconds>(now_ms);
        // failedPath.utime = value.count();
        
        failedPath.waypoints.push_back(start);

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


Path MotionPlanner::planPath(const Waypoint& start, const Waypoint& goal) const
{
    return planPath(start, goal, searchParams_);
}

bool MotionPlanner::isValidStart(const Waypoint& start) const
{
    
    Vector2i startCell = global_position_to_grid_cell(Vector2d(start.position(0), 
        start.position(1)), distances_);
   
    
    // A valid goal is in the grid
    if(distances_.isCellInGrid(startCell(0), startCell(1)))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(startCell(0), startCell(1)) > params_.robotRadius;
    }
    
    // A start point must be in the map for the robot to reach it
    printf("This start point is invalid.\n");
    return false;
}

bool MotionPlanner::isValidGoal(const Waypoint& goal) const
{
    
    Vector2i goalCell = global_position_to_grid_cell(Vector2d(goal.position(0), 
        goal.position(1)), distances_);
   
    
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell(0), goalCell(1)))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(goalCell(0), goalCell(1)) > params_.robotRadius;
    }
    
    // A goal must be in the map for the robot to reach it
    printf("This goal is invalid.\n");
    return false;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}

} // namespace gnc
} // namespace maav