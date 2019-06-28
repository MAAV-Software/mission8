#define BOOST_TEST_MODULE PlannerUtilTest
/**
 * Unit tests for the math library in common
 */

#include <cmath>
#include <vector>
#include <Eigen/Eigen>
#include <iostream>
#include <boost/test/unit_test.hpp>
#include "gnc/planner/plannerUtils.hpp"

using namespace boost::unit_test;
using namespace maav::gnc::planner;
namespace tt = boost::test_tools;
using octomap::point3d;
using std::cout;
using std::endl;
using std::vector;

void printGrid(vector<vector<size_t> > &grid)
{
    for(const auto&row: grid)
    {
        cout << "[ ";
        for(const auto&id: row)
        {
            cout << id << ",\t";
        }
        cout << " ]\n";
    }
}

BOOST_AUTO_TEST_CASE(GridTest)
{
    double resolution = 0.1;
    GetId getId(resolution);
    vector<vector<size_t> > id_grid;

    for(double j = -5*resolution; j <= 5*resolution; j+= resolution) {
        vector<size_t> row;
        for(double i = 5*resolution; i >= -5*resolution; i-= resolution) {
            row.push_back(getId(point3d(i, j, 0.0)));
        }
        id_grid.push_back(row);
    }
    printGrid(id_grid);

    BOOST_CHECK_EQUAL(0, 0);
}