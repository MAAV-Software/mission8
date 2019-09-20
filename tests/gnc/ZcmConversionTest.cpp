#define BOOST_TEST_MODULE ZcmConversionTest

#include <boost/test/unit_test.hpp>

#include <gnc/utils/ZcmConversion.hpp>

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc;
using namespace maav::gnc::measurements;

BOOST_AUTO_TEST_CASE(StateConversionTest)
{
    State state;
    state.position() = {0.1, 0.2, 0.3};
    state.velocity() = {0.2323, -0.3423, -0.4329};
    state.attitude().setQuaternion({0, 0, 1, 0});
    state.gravity() = {0, 0, -9.8};
    state.gyroBias() = {0.02, 0.002, -0.000923};
    state.accelBias() = {-0.0032, 0.00023, 0.0423};
    state.setTime(1231231231231);
    state.covariance() = State::CovarianceMatrix::Identity() * -0.89238928392739;

    const auto state2 = ConvertState(state);
    const State state3 = ConvertState(state2);

    BOOST_CHECK_EQUAL(state.attitude().matrix(), state3.attitude().matrix());
    BOOST_CHECK_EQUAL(state.position(), state3.position());
    BOOST_CHECK_EQUAL(state.velocity(), state3.velocity());
    BOOST_CHECK_EQUAL(state.covariance(), state3.covariance());

    BOOST_CHECK_EQUAL(state.timeUSec(), state2.utime);
    BOOST_CHECK_EQUAL(state.timeUSec(), state3.timeUSec());
    BOOST_CHECK_EQUAL(state.gyroBias(), state3.gyroBias());
    BOOST_CHECK_EQUAL(state.accelBias(), state3.accelBias());
    BOOST_CHECK_EQUAL(state.acceleration(), state3.acceleration());
    BOOST_CHECK_EQUAL(state.angularVelocity(), state3.angularVelocity());
    BOOST_CHECK_EQUAL(state.magneticFieldVector(), state3.magneticFieldVector());
    BOOST_CHECK_EQUAL(state.gravity(), state3.gravity());
}

BOOST_AUTO_TEST_CASE(WaypointConversionTest)
{
    waypoint_t w1;
    w1.pose[0] = 0.2312;
    w1.pose[1] = 123.232;
    w1.pose[2] = -923.232;
    w1.pose[3] = -0.0000001;

    w1.rate[0] = 0.232;
    w1.rate[1] = -23.12;
    w1.rate[2] = -2.00009;

    Waypoint w2 = ConvertWaypoint(w1);

    BOOST_CHECK_EQUAL(w1.pose[0], w2.position.x());
    BOOST_CHECK_EQUAL(w1.pose[1], w2.position.y());
    BOOST_CHECK_EQUAL(w1.pose[2], w2.position.z());
    BOOST_CHECK_EQUAL(w1.pose[3], w2.yaw);

    BOOST_CHECK_EQUAL(w1.rate[0], w2.velocity.x());
    BOOST_CHECK_EQUAL(w1.rate[1], w2.velocity.y());
    BOOST_CHECK_EQUAL(w1.rate[2], w2.velocity.z());
}

BOOST_AUTO_TEST_CASE(ImuConversionTest)
{
    imu_t i1;
    i1.utime = 2392328392;
    i1.acceleration.data[0] = 0.2312;
    i1.acceleration.data[1] = 123.232;
    i1.acceleration.data[2] = -923.232;

    i1.angular_rates.data[0] = 0.223312;
    i1.angular_rates.data[1] = 12233.232;
    i1.angular_rates.data[2] = -92223.232;

    i1.magnetometer.data[0] = 0.232312;
    i1.magnetometer.data[1] = 123.2332;
    i1.magnetometer.data[2] = -923.212332;

    auto i2 = convertImu(i1);

    BOOST_CHECK_EQUAL(i1.utime, i2->time_usec);

    BOOST_CHECK_EQUAL(i1.angular_rates.data[0], i2->angular_rates.x());
    BOOST_CHECK_EQUAL(i1.angular_rates.data[1], i2->angular_rates.y());
    BOOST_CHECK_EQUAL(i1.angular_rates.data[2], i2->angular_rates.z());

    BOOST_CHECK_EQUAL(i1.acceleration.data[0], i2->acceleration.x());
    BOOST_CHECK_EQUAL(i1.acceleration.data[1], i2->acceleration.y());
    BOOST_CHECK_EQUAL(i1.acceleration.data[2], i2->acceleration.z());

    BOOST_CHECK_EQUAL(i1.magnetometer.data[0], i2->magnetometer.x());
    BOOST_CHECK_EQUAL(i1.magnetometer.data[1], i2->magnetometer.y());
    BOOST_CHECK_EQUAL(i1.magnetometer.data[2], i2->magnetometer.z());
}

BOOST_AUTO_TEST_CASE(LidarConversionTest)
{
    lidar_t l1;
    l1.utime = 1231231;
    l1.distance.data[0] = 98392.2323;

    auto l2 = convertLidar(l1);

    BOOST_CHECK_EQUAL(l1.utime, l2->timeUSec());
    BOOST_CHECK_EQUAL(l1.distance.data[0], l2->distance()(0));
}

BOOST_AUTO_TEST_CASE(PlaneFitConversionTest)
{
    plane_fit_t p1;

    p1.utime = 98928392;
    p1.roll.data[0] = 0.2323;
    p1.pitch.data[0] = 0.2832;
    p1.z.data[0] = -0.99;
    p1.z_dot.data[0] = -0.0023;

    auto p2 = convertPlaneFit(p1);

    BOOST_CHECK_EQUAL(p1.utime, p2->time_usec);
    BOOST_CHECK_EQUAL(p1.roll.data[0], p2->roll);
    BOOST_CHECK_EQUAL(p1.pitch.data[0], p2->pitch);
    BOOST_CHECK_EQUAL(p1.z.data[0], p2->height);
    BOOST_CHECK_EQUAL(p1.z_dot.data[0], p2->vertical_speed);
}

BOOST_AUTO_TEST_CASE(GlobalUpdateConversionTest)
{
    global_update_t g1;

    g1.utime = 92839283;
    g1.position.data[0] = 93842.34234;
    g1.position.data[1] = 42.3423984954;
    g1.position.data[2] = 942.323234234;

    g1.attitude.data[0] = 0;
    g1.attitude.data[1] = 0;
    g1.attitude.data[2] = 1;
    g1.attitude.data[3] = 0;

    auto g2 = convertGlobalUpdate(g1);

    BOOST_CHECK_EQUAL(g1.utime, g2->timeUSec());

    BOOST_CHECK_EQUAL(g1.position.data[0], g2->pose().translation()[0]); // x
    BOOST_CHECK_EQUAL(g1.position.data[1], g2->pose().translation()[1]); // y
    BOOST_CHECK_EQUAL(g1.position.data[2], g2->pose().translation()[2]); // z

    const Eigen::Quaterniond& q = g2->pose().unit_quaternion();
    BOOST_CHECK_EQUAL(g1.attitude.data[0], q.w());
    BOOST_CHECK_EQUAL(g1.attitude.data[1], q.x());
    BOOST_CHECK_EQUAL(g1.attitude.data[2], q.y());
    BOOST_CHECK_EQUAL(g1.attitude.data[3], q.z());
}
