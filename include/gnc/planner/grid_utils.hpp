#ifndef MAPPING_OCCUPANCY_GRID_UTILS_HPP
#define MAPPING_OCCUPANCY_GRID_UTILS_HPP

#include <gnc/planner/occupancy_grid.hpp>
#include <Eigen/Eigen>

/**
* grid_position_to_global_position converts a point in the grid coordinate system to the global coordinate system.
*
* \param    gridPosition    Position to be converted
* \param    grid            OccupancyGrid in which gridPoint exists
* \return   Point in the global coordinate frame.
*/
template <class Grid>
Eigen::Vector2d grid_position_to_global_position(const Eigen::Vector2d& gridPosition, const Grid& grid)
{
    return Eigen::Vector2d(grid.originInGlobalFrame()[0] + gridPosition[0]*grid.metersPerCell(),
                         grid.originInGlobalFrame()[1] + gridPosition[1]*grid.metersPerCell());
}


/**
* global_position_to_grid_cell converts a point in the global coordinate system to a cell in the grid coordinate system.
* 
* The global point likely falls somewhere within a cell in the grid. This function truncates the calculation
* to return the origin of the cell in which the point falls. If fractions of a cell are important -- say for ray
* tracing -- then global_position_to_grid_position should be called instead, as it allows for fractions of a cell.
*
* \param    globalPosition  Point to be converted
* \param    grid            CellOccupancyGrid in which globalPosition will be converted
* \return   Cell in the grid coordinate frame.
*/
template <class Grid>
Eigen::Vector2i global_position_to_grid_cell(const Eigen::Vector2d& globalPosition, const Grid& grid)
{
    return Eigen::Vector2i(static_cast<int>((globalPosition[0] - grid.originInGlobalFrame()[0]) * grid.cellsPerMeter()),
                      static_cast<int>((globalPosition[1] - grid.originInGlobalFrame()[1]) * grid.cellsPerMeter()));
}


/**
* global_position_to_grid_position converts a point in the global coordinate system to a point in the grid coordinate system. The
* point can be fractions of a grid cell so no information (modulo floating-point error) is lost converting back and forth
* between global and grid points.
*
* \param    globalPosition  Point to be converted
* \param    grid            CellOccupancyGrid in which globalPosition will be converted
* \return   Point in the grid coordinate frame.
*/
template <class Grid>
Eigen::Vector2d global_position_to_grid_position(const Eigen::Vector2d& globalPosition, const Grid& grid)
{
    return Eigen::Vector2d((globalPosition[0] - grid.originInGlobalFrame()[0]) * grid.cellsPerMeter(),
                         (globalPosition[1] - grid.originInGlobalFrame()[1]) * grid.cellsPerMeter());
}

#endif // MAPPING_OCCUPANCY_GRID_UTILS_HPP
