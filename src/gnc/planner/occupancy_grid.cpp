#include <gnc/planner/occupancy_grid.hpp>
#include <fstream>
#include <cassert>
#include <iostream>

namespace maav
{
namespace gnc
{

OccupancyGrid::OccupancyGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(1.0 / metersPerCell_)
, globalOrigin_(0, 0)
{
}


OccupancyGrid::OccupancyGrid(float widthInMeters,
                             float heightInMeters,
                             float metersPerCell)
: metersPerCell_(metersPerCell)
, globalOrigin_(-widthInMeters/2.0f, -heightInMeters/2.0f)
{
    assert(widthInMeters  > 0.0f);
    assert(heightInMeters > 0.0f);
    assert(metersPerCell_ <= widthInMeters);
    assert(metersPerCell_ <= heightInMeters);
    
    cellsPerMeter_ = 1.0f / metersPerCell_;
    width_         = widthInMeters * cellsPerMeter_;
    height_        = heightInMeters * cellsPerMeter_;
    
    cells_.resize(width_ * height_);
    reset();
}


void OccupancyGrid::reset(void)
{
    std::fill(cells_.begin(), cells_.end(), 0);
}


bool OccupancyGrid::isCellInGrid(int x, int y) const
{ 
    bool xCoordIsValid = (x >= 0) && (x < width_);
    bool yCoordIsValid = (y >= 0) && (y < height_);
    return xCoordIsValid && yCoordIsValid;
}


CellOdds OccupancyGrid::cellOdds(int x, int y) const
{
    if(isCellInGrid(x, y))
    {
        return operator()(x, y);
    }
    
    return 0;
}


void OccupancyGrid::setCellOdds(int x, int y, CellOdds value)
{
    if(isCellInGrid(x, y))
    {
        operator()(x, y) = value;
    }
}


occupancy_grid_t OccupancyGrid::toZCM(void) const
{
    occupancy_grid_t grid;

    grid.origin_x        = static_cast<double>(globalOrigin_[0]);
    grid.origin_y        = static_cast<double>(globalOrigin_[1]);
    grid.resolution = metersPerCell_;
    grid.width           = width_;
    grid.height          = height_;
    grid.num_cells       = cells_.size();
    grid.cells           = cells_;
    
    return grid;
}


void OccupancyGrid::fromZCM(const occupancy_grid_t& gridMessage)
{
    globalOrigin_[0] = static_cast<float>(gridMessage.origin_x);
    globalOrigin_[1] = static_cast<float>(gridMessage.origin_y);
    metersPerCell_  = gridMessage.resolution;
    cellsPerMeter_  = 1.0f / gridMessage.resolution;
    height_         = gridMessage.height;
    width_          = gridMessage.width;
    cells_          = gridMessage.cells;
}


bool OccupancyGrid::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);
    if(!out.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::saveToFile: Failed to save to " << filename << '\n';
        return false;
    }
    
    // Write header
    out << globalOrigin_[0] << ' ' << globalOrigin_[1] << ' ' << width_ << ' ' << height_ << ' ' << metersPerCell_ << '\n';
    
    // Write out each cell value
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            // Unary plus forces output to be a a number rather than a character
             out << +cellOdds(x, y) << ' ';
        }
        out << '\n';
    }
    
    return out.good();
}


bool OccupancyGrid::loadFromFile(const std::string& filename)
{
    std::ifstream in(filename);
    if(!in.is_open())
    {
        std::cerr << "ERROR: OccupancyGrid::loadFromFile: Failed to load from " << filename << '\n';
        return false;
    }
    
    width_ = -1;
    height_ = -1;
    
    // Read header
    in >> globalOrigin_[0] >> globalOrigin_[1] >> width_ >> height_ >> metersPerCell_;
    
    // Check sanity of values
    assert(width_ > 0);
    assert(height_ > 0);
    assert(metersPerCell_ > 0.0f);
    
    // Allocate new memory for the grid
    cells_.resize(width_ * height_);
    // Read in each cell value
    int odds = 0; // read in as an int so it doesn't convert the number to the corresponding ASCII code
    for(int y = 0; y < height_; ++y)
    {
        for(int x = 0; x < width_; ++x)
        {
            in >> odds;
            setCellOdds(x, y, odds);
        }
    }
    
    return true;
}
} // namespace gnc
} // namespace maav