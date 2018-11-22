#define BOOST_TEST_MODULE "PidTest"
#include <boost/test/unit_test.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include "gnc/control/pid.hpp"

using std::cout;
using std::endl;
using std::abs;
using std::vector;
using std::ifstream;
using maav::gnc::control::Pid;

class Fixture
{
public:
    Fixture()
        : pidZeros(), pidPreset(Pid(1.00, 0.01, 0.10)), p(1.00), i(0.01), d(0.10), tol(0.0000001)
    {
    }

    double absDiff(double a, double b) { return abs(a - b); }
    Pid pidZeros;
    Pid pidPreset;
    double p;
    double i;
    double d;
    double tol;
};

BOOST_AUTO_TEST_CASE(initTest)
{
    Fixture f;

    BOOST_CHECK(f.absDiff(f.pidZeros.run(0.0, 0.0), 0.0) < f.tol);
    BOOST_CHECK(f.absDiff(f.pidZeros.runDiscrete(1.0, 1.0), 0.0) < f.tol);

    BOOST_CHECK(f.absDiff(f.pidPreset.runDiscrete(0.0, 1.0), 0.0) < f.tol);
    BOOST_CHECK(f.absDiff(f.pidPreset.run(0.0, 0.0), 0.0) < f.tol);
}

BOOST_AUTO_TEST_CASE(presetGainsTest)
{
    Fixture f;
    double ans = 1.11;
    double res = f.pidPreset.run(1.0, 1.0);
    BOOST_CHECK(f.absDiff(res, ans) < f.tol);
}

BOOST_AUTO_TEST_CASE(setGainsTest)
{
    Fixture f;

    f.pidZeros.setGains(f.p, f.i, f.d);
    BOOST_CHECK(f.absDiff(f.pidZeros.run(1.0, 1.0), f.pidPreset.run(1.0, 1.0)) < f.tol);
}

BOOST_AUTO_TEST_CASE(twoIterAndResetTest)
{
    Fixture f;
    constexpr double ans1 = 1.11;
    constexpr double ans2 = 1.12;

    BOOST_CHECK(f.absDiff(f.pidPreset.run(1.0, 1.0), ans1) < f.tol);
    BOOST_CHECK(f.absDiff(f.pidPreset.run(1.0, 1.0), ans2) < f.tol);

    f.pidPreset.reset();
    BOOST_CHECK(f.absDiff(f.pidPreset.run(1.0, 1.0), ans1) < f.tol);
}

BOOST_AUTO_TEST_CASE(discreteMultiIterTest)
{
    Fixture f;
    double e = 5.0;
    size_t iters = 100;
    double dt = 0.1;
    double edot = 2.0 * e / static_cast<double>(iters);
    vector<double> error(iters, 0.0);
    for (double& x : error)
    {
        x = e;
        e -= edot;
    }

    vector<double> ans(iters, 0);
    ifstream file("pid_res.out", std::ifstream::in);
    double cnt = 0;
    while (file.is_open() && file.good() && !file.eof()) file >> ans[cnt++];

    file.close();

    BOOST_CHECK_EQUAL(cnt, iters);

    for (size_t i = 0; i < iters; ++i)
    {
        double res = f.pidPreset.runDiscrete(error[i], dt);
        BOOST_CHECK(f.absDiff(res, ans[i]) < f.tol);
    }
}
