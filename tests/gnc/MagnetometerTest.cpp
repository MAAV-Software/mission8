#define BOOST_TEST_MODULE MagnetometerTest

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/output_test_stream.hpp>
#include <boost/test/unit_test.hpp>

#include <gnc/utils/MagnetometerEllipsoidFit.hpp>

#include "TestHelpers.hpp"

using std::ifstream;
using std::vector;

using namespace boost::unit_test;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(MagnetometerTest)
{
    ifstream fin("EllipsoidPointCloud.txt");

    vector<vector<double>> vals;

    double xx, yy, zz;
    while (fin >> xx >> yy >> zz)
    {
        vals.push_back(vector<double>{xx, yy, zz});
    }

    VectorXd x(vals.size());
    VectorXd y(vals.size());
    VectorXd z(vals.size());

    int idx = 0;
    for (auto &v : vals)
    {
        x(idx) = v[0];
        y(idx) = v[1];
        z(idx) = v[2];
        idx++;
    }

    MagParams mp = maav::gnc::runMagnetometerCalibration(x, y, z);

    Matrix3d rotM_correct;
    rotM_correct << 0.281281255050245, -0.0300587933143593, -0.959154484169179, -0.604265495276467,
        0.770918159692133, -0.201366338485811, 0.745482438945175, 0.636224535837294,
        0.198681335869714;

    MatrixXd rotM_t = rotM_correct.transpose();

    for (int i = 0; i < 9; i++)
    {
        BOOST_CHECK_CLOSE(rotM_t(i), mp.rotM(i), 0.001);
    }
}