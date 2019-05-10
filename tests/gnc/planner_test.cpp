#define BOOST_TEST_MODULE PlannerTest
/**
 * Tests for AStar and Motion planner
 */
#include <cmath>
#include <vector>
#include <iostream>

#include "gnc/measurements/Waypoint.hpp"
#include "gnc/planner/occupancy_grid.hpp"
#include "gnc/planner/Astar.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

using namespace boost::unit_test;
using namespace Eigen;
using std::vector;
using std::cout;
using std::endl;

using namespace maav::gnc::planner;
using namespace maav::gnc;
State identity(const State& state) { return state; }

// make grid creation easier
// Grid convention: the 1's represent an obstacle, 0's is a free space,
// 4: goal, 3: start
// waypoints position = {x:col, y:row, 0}
OccupancyGrid getGridInfo(Waypoint& start, Waypoint& goal, vector<vector<int> > grid)
{
	OccupancyGrid oGrid;
	occupancy_grid_t zcmMessage;
	// must create a zcm message first
	zcmMessage.width = grid[0].size();
	zcmMessage.height = grid.size();
	zcmMessage.num_cells = zcmMessage.width*zcmMessage.height;
	zcmMessage.origin_x = 0;
	zcmMessage.origin_y = 0;
	zcmMessage.resolution = 1;
	zcmMessage.cells.resize(zcmMessage.num_cells, 0);
	for(size_t i = 0; i < grid.size(); ++i)
	{
		for(size_t j = 0; j < grid[i].size(); ++j)
		{
			// start block
			if(grid[i][j] == 3)
			{
				start.position = Vector3d(j*zcmMessage.resolution, 
										  i*zcmMessage.resolution, 0.);
				start.velocity = Vector3d(0.,0.,0.);
				start.yaw = 0;
				zcmMessage.cells[i*grid[i].size() + j] = 0;
			}
			// goal block
			else if(grid[i][j] == 4)
			{
				goal.position = Vector3d(j*zcmMessage.resolution, 
										 i*zcmMessage.resolution, 0.);
				goal.velocity = Vector3d(0.,0.,0.);
				goal.yaw = 0;
				zcmMessage.cells[i*grid[i].size() + j] = 0;
			}
			else
			{
				zcmMessage.cells[i*grid[i].size() + j] = grid[i][j];
			}
		}
	}

	oGrid.fromZCM(zcmMessage);
	return oGrid;
}

// Ensure that the occupancy grid zcm types are created correctly
BOOST_AUTO_TEST_CASE(OccupancyGridCreation)
{
	vector<vector<int> > grid{ 
								{ 0, 4, 1, 0, 0 }, 
								{ 0, 0, 1, 0, 0 }, 
								{ 1, 0, 1, 1, 0 },
								{ 1, 0, 0, 0, 0 },
								{ 1, 1, 1, 1, 0 },
								{ 0, 0, 3, 0, 0 },
								{ 0, 0, 0, 0, 0 } }; 
    Waypoint start;
    Waypoint goal;
	OccupancyGrid oGrid = getGridInfo(start, goal, grid);

	// ensure that the oGrid was created correctly
	BOOST_CHECK_EQUAL(oGrid.widthInCells(), grid[0].size());
	BOOST_CHECK_EQUAL(oGrid.heightInCells(), grid.size());
	auto origin = oGrid.originInGlobalFrame();
	BOOST_CHECK_EQUAL(origin, Eigen::Vector2f(0.,0.));

	// check that all the cells correspond correctly in the grid
	for(size_t y = 0; y < grid.size(); ++y)
	{
		for(size_t x = 0; x < grid[y].size(); ++x)
		{
			if(grid[y][x] == 1)
			{
				BOOST_CHECK_EQUAL(oGrid(x, y), 1);
			}
			else 
			{
				BOOST_CHECK_EQUAL(oGrid(x, y), 0);
			}
		}
	}
}

void printPath(const Path& p) 
{
	for(auto w: p.waypoints)
	{
		cout << w.position.x() << ", " <<  w.position.y() << "\n";
	}
}
// Test a simple Astar path
BOOST_AUTO_TEST_CASE(AStarPathSimple)
{
	vector<vector<int> > grid{ 
							   { 0, 4, 1, 0, 0 }, 
		                       { 0, 0, 1, 0, 0 }, 
		                       { 1, 0, 1, 1, 0 },
		                       { 1, 0, 0, 0, 0 },
		                       { 1, 1, 1, 1, 0 },
		                       { 0, 0, 3, 0, 0 },
		                       { 0, 0, 0, 0, 0 } }; 
    Waypoint start;
    Waypoint goal;
	OccupancyGrid oGrid = getGridInfo(start, goal, grid);
	Astar astar;
	Path path = astar(start, goal, oGrid);

	// we have a vector of waypoints. we must compare the positions
	 
	
							   // 0{ 0, 4, 1, 0, 0 }, 
		        //                1{ 0, 0, 1, 0, 0 }, 
		        //                2{ 1, 0, 1, 1, 0 },
		        //                3{ 1, 0, 0, 0, 0 },
		        //                4{ 1, 1, 1, 1, 0 },
		        //                5{ 0, 0, 3, 0, 0 },
		        //                6{ 0, 0, 0, 0, 0 } };
		        //                   0  1  2  3  4 
	// (5,2) (5,3) (4,4) (3,3) (3,2), (2,1) (1,1) (0,1)

	//vector<vector<int> > truePath { }
	//printPath(path);

	// Create a way to ensure that it is correct
	Vector3d vel(0,0,0);
	std::vector<Waypoint> truePath = {
		Waypoint( Vector3d(3,5,0), vel, 0),
		Waypoint( Vector3d(4,4,0), vel, 0),
		Waypoint( Vector3d(3,3,0), vel, 0),
		Waypoint( Vector3d(2,3,0), vel, 0),
		Waypoint( Vector3d(1,2,0), vel, 0),
		Waypoint( Vector3d(1,1,0), vel, 0),
		Waypoint( Vector3d(1,0,0), vel, 0) 
	};

	for(unsigned i = 0; i < truePath.size(); ++i)
	{
		BOOST_CHECK_EQUAL(truePath[i], path[i]);
	}
}
