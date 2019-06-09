#define BOOST_TEST_MODULE MathTest
/**
 * Unit tests for the math library in common
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include "common/math/math.hpp"

using namespace boost::unit_test;
using namespace Eigen;
using namespace maav;
namespace tt = boost::test_tools;

BOOST_AUTO_TEST_CASE(YawTest)
{
    // test all 4 qaudrants for yaw heading
    double north = rad_to_deg(yaw_between(Vector3d(1,0,0), Vector3d(1,1,0)));
    double east = rad_to_deg(yaw_between(Vector3d(-1,0,0), Vector3d(1,0,0)));
    double south = rad_to_deg(yaw_between(Vector3d(1,0,0), Vector3d(1,-1,0)));
    double west = rad_to_deg(yaw_between(Vector3d(0,1,0), Vector3d(-1,1,0)));
    double SE = rad_to_deg(yaw_between(Vector3d(-1,0,0), Vector3d(0,-1,0)));
    double SW = rad_to_deg(yaw_between(Vector3d(1,0,0), Vector3d(0,-1,0)));
    double NW = rad_to_deg(yaw_between(Vector3d(1,0, 0), Vector3d(0,1,0)));
    double NE = rad_to_deg(yaw_between(Vector3d(-1,0, 0), Vector3d(0,1,0)));

    BOOST_CHECK_CLOSE(north, 360, 0.001);
    BOOST_CHECK_CLOSE(east, 90, 0.001); 
    BOOST_CHECK_CLOSE(south, 180, 0.001);
    BOOST_CHECK_CLOSE(west, 270, 0.001);
    BOOST_CHECK_CLOSE(SE, 135, 0.001);
    BOOST_CHECK_CLOSE(SW, 225, 0.001);
    BOOST_CHECK_CLOSE(NW, 315, 0.001);
    BOOST_CHECK_CLOSE(NE, 45, 0.001);

    // edge cases 
    double same = rad_to_deg(yaw_between(Vector3d(1,3,12), Vector3d(2,6,0)));
    double x = rad_to_deg(yaw_between(Vector3d(0,0,0), Vector3d(2,6,0)));

    BOOST_CHECK_EQUAL(same, x);
}