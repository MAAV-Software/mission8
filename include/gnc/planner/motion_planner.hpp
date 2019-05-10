#ifndef PLANNING_MOTION_PLANNER_HPP
#define PLANNING_MOTION_PLANNER_HPP

#include <common/messages/path_t.hpp>
#include <gnc/measurements/Waypoint.hpp>
#include <gnc/planner/astar.hpp>
#include <gnc/planner/obstacle_distance_grid.hpp>
#include <gnc/planner/path.hpp>

namespace maav
{
namespace gnc
{
/**
* MotionPlannerParams defines the parameters that control the behavior of the motion planner.
*/
struct MotionPlannerParams
{
    double robotRadius;     ///< Radius of the robot for which paths are being planned

    /**
    * Default constructor for MotionPlannerParams.
    * 
    * Assign default values that are suitable for successful motion planning for the MAEbot.
    * 
    * The minimum valid radius is 0.05, as that is the radius of the MAEbot.
    * 
    * The planner will remain robotRadius + metersPerCell from walls to keep the robot from getting sucked into nearby
    * walls.
    */
    MotionPlannerParams(void)
    : robotRadius(0.09) // by default, have a little extra slop to keep the robot from getting too close to the walls
    {
    }
};


/**
* MotionPlanner is simple motion planning implementation for MAEbots. The MotionPlanner uses an A* search to find the
* shortest path to a goal in the MAEbot's configuration space.
* 
* To use the motion planner:
* 
*   - Set the current map of the environment via the setMap method. Whenever the occupancy grid changes, you'll need to
*     call setMap again for the planner to include the updates to the grid.
*   - Select a valid goal pose using the isValidGoal method, which confirms that the goal isn't too close to a wall
*     for the robot to reach
*   - Find a path to the goal using the planPath method. Use the robot's current estimated pose as the start.
*/
class MotionPlanner
{
public:

    /**
    * Constructor for MotionPlanner.
    *
    * \param    params          Parameters for controlling the behavior of the planner (optional)
    */
    explicit MotionPlanner(const MotionPlannerParams& params = MotionPlannerParams());
    
    /**
    * Constructor for MotionPlanner.
    *
    * \param    params          Parameters for controlling the behavior of the planner (optional)
    * \param    searchParams    A*-search parameters to fine-tune the behavior of the planner
    */

    MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams);

    /**
    * planPath performs an A* search through the robot's configuration space to find a path from the given start pose
    * to the goal pose.
    * 
    * The returned path can be navigated by driving to the next pose in the path, turning in place to face the next
    * pose, driving to it, and repeating. 
    * 
    * If the start or goal is not valid or the goal is unreachable because there's not enough room for the robot, then
    * the planner will fail to find a path. In this case, a path containing only the start pose is returned.
    * 
    * The path found attempts to minimize the number of poses in the path by reducing the cell-by-cell path to a sequence
    * of poses at the endpoints of lines defined by the path's cells. If the returned path still has too many turns,
    * additional poses can be removed by checking if skipping them doesn't result in the robot hitting a wall.
    * 
    * \param    start           Starting pose for the path
    * \param    goal            Goal pose for the path
    * \param    searchParams    Parameters to provide to the A* planner to fine-tune its behavior
    * \return   Path found from start to end. If no path is found, then the path length is 1 and contains only the start
    *   pose.
    */
    Path planPath(const Waypoint& start, const Waypoint& goal, const SearchParams& searchParams) const;
    
    /**
    * planPath is a simplified version of the planPath method that SearchParams provided during construction of the
    * MotionPlanner. Unless you are doing some some of adaptive planning strategy, you can use this method all the time
    * to find paths through the map.
    * 
    * \param    start           Starting pose for the path
    * \param    goal            Goal pose for the path
    * \return   Path found from start to end. If no path is found, then the path length is 1 and contains only the start
    *   pose.
    */
    Path planPath(const Waypoint& start, const Waypoint& goal) const;

    /**
    * isValidGoal checks if the robot can possibly reach the specified goal pose. A valid goal is one that is at least
    * one robot radius from any known obstacle.
    *
    * A goal being valid does not guarantee that a path exists to the goal, it just means that a path might exist. Thus,
    * if presented with a number of different goals for motion planning, isValidGoal can be used as an initial filter
    * for determining which goal to select for a call to planPath.
    *
    * \param    goal            Desired goal pose for a planned path
    * \return   True if goal is at least one robot radius away from the nearest obstacle.
    */
    bool isValidGoal(const Waypoint& goal) const;
    bool isValidStart(const Waypoint& start) const;
    
    /**
    * setMap sets the map for which path's will be planned.
    *
    * \param    map         OccupancyGrid representation of the environment through which paths will be planned
    */
    void setMap(const OccupancyGrid& map);
    
    /**
    * setParams changes the default search parameters used by the motion planner.
    * 
    * \param    params          New parameters for the motion planner
    */
    void setParams(const MotionPlannerParams& params);

    /**
    * obstacleDistances retrieves the ObstacleDistanceGrid used during motion planning. The distances are intended for
    * debugging use to create a visualization of the configuration space of the environment.
    * 
    * \return   ObstacleDistanceGrid currently being used by the motion planner.
    */
    ObstacleDistanceGrid obstacleDistances(void) const { return distances_; }

private:
    
    ObstacleDistanceGrid distances_;
    MotionPlannerParams params_;
    SearchParams searchParams_;
};
}
}
#endif // PLANNING_MOTION_PLANNER_HPP
