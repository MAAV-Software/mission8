#define BOOST_TEST_MODULE MathTest
/**
 * Unit tests for the math library in common
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include "common/math/math.hpp"
#include "common/math/angle_functions.hpp"

using namespace boost::unit_test;
using namespace Eigen;
using namespace maav;
namespace tt = boost::test_tools;

BOOST_AUTO_TEST_CASE(YawTest)
{
    // test all 4 qaudrants for yaw heading
    double east = yaw_between(Vector3d(1,0,0), Vector3d(1,1,0));
    double north = yaw_between(Vector3d(-1,0,0), Vector3d(1,0,0));
    double west = yaw_between(Vector3d(1,0,0), Vector3d(1,-1,0));
    double south = yaw_between(Vector3d(0,1,0), Vector3d(-1,1,0));
    double NW = yaw_between(Vector3d(-1,0,0), Vector3d(0,-1,0));
    double SW = yaw_between(Vector3d(1,0,0), Vector3d(0,-1,0));
    double SE = yaw_between(Vector3d(1,0, 0), Vector3d(0,1,0));
    double NE = yaw_between(Vector3d(-1,0, 0), Vector3d(0,1,0));

    BOOST_CHECK_SMALL(eecs467::angle_diff(north, 2*PI), 0.0001);
    BOOST_CHECK_CLOSE(east, PI/2.0, 0.001); 
    BOOST_CHECK_CLOSE(south, PI, 0.001);
    BOOST_CHECK_CLOSE(west, 3*PI/2, 0.001);
    BOOST_CHECK_CLOSE(SE, 3*PI/4, 0.001);
    BOOST_CHECK_CLOSE(SW, 5*PI/4, 0.001);
    BOOST_CHECK_CLOSE(NW, 7*PI/4, 0.001);
    BOOST_CHECK_CLOSE(NE, PI/4, 0.001);
    // edge cases 
    double same = rad_to_deg(yaw_between(Vector3d(1,3,12), Vector3d(2,6,0)));
    double x = rad_to_deg(yaw_between(Vector3d(0,0,0), Vector3d(2,6,0)));

    BOOST_CHECK_EQUAL(same, x);
}