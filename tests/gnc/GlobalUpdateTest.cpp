#define BOOST_TEST_MODULE GlobalUpdateTest
/**
 * Tests for both BaseState and Kalman State
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <gnc/Constants.hpp>
#include <gnc/kalman/updates/GlobalUpdate.hpp>
#include "TestHelpers.hpp"

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::kalman;
using namespace maav::gnc;
using namespace maav::gnc::measurements;

BOOST_AUTO_TEST_CASE(SimpleSensorModelTest)
{
    State state(1000);
    state.attitude() = Sophus::SO3d(Quaterniond{1, 0, 0, 0});
    state.position() = {0, 1, -1};
    state.velocity() = Vector3d::Zero();
    state.angularVelocity() = Vector3d::Zero();
    state.acceleration() = Vector3d::Zero();

    GlobalUpdate update(YAML::Load(
        "  global_update:\n    enabled: true\n    enable_outliers: false\n    # Unscented "
        "transform params\n    "
        "UT:\n      alpha: 0.1\n      beta: 2.0\n      kappa: 0.0\n    # Sensor noise covariance\n "
        "   R: [0.000001, 0.000001, 0.000001, 0.0000001, 0.0000001, 0.0000001]\n    extrinsics:\n  "
        "    rot: [1, 0, 0, 0]\n      pos: [0, 0, 0]"));

    auto pred = update.predicted(state);
    BOOST_CHECK_LE((pred.pose().translation() - state.position()).norm(), 0.000001);
    BOOST_CHECK_LE(diff(pred.pose().so3(), state.attitude()), 0.000001);
}

BOOST_AUTO_TEST_CASE(AdvancedSensorModelTest)
{
    State state(1000);
    state.attitude() = Sophus::SO3d(Quaterniond{0.982, 0, 0.191, 0});
    state.attitude().normalize();
    state.position() = {10.1, -23.2, -3.25};
    state.velocity() = Vector3d::Zero();
    state.angularVelocity() = Vector3d::Zero();
    state.acceleration() = Vector3d::Zero();

    GlobalUpdate update(
        YAML::Load("  global_update:\n    enabled: true\n    enable_outliers: false\n    # "
                   "Unscented transform params\n    "
                   "UT:\n      alpha: 0.1\n      beta: 2.0\n      kappa: 0.0\n    # Sensor noise "
                   "covariance\n    R: [0.000001, 0.000001, 0.000001, 0.0000001, 0.0000001, "
                   "0.0000001]\n    extrinsics:\n      rot: [1, 0, 0, 0]\n      pos: [0, 0, 0]"));

    auto pred = update.predicted(state);
    BOOST_CHECK_LE((pred.pose().translation() - state.position()).norm(), 0.000001);
    BOOST_CHECK_LE(diff(pred.pose().so3(), state.attitude()), 0.000001);
}
