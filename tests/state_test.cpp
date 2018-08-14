#define BOOST_TEST_MODULE StateTest

#include <boost/test/unit_test.hpp>

#include "gnc/kalman/kalman_state.hpp"
#include "gnc/state/base_state.hpp"

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::state;
using namespace maav::gnc::kalman;

double diff(const Quaterniond& q1, const Quaterniond& q2) {
    const Quaterniond q3 = q1 * q2.conjugate();
    return acos(q3.w());
}

BOOST_AUTO_TEST_CASE(ZeroBaseStateTest) {
    BaseState base = BaseState::zero(0);

    double quaternion_error =
        diff(base.attitude().unit_quaternion(), Quaterniond::Identity());
    BOOST_REQUIRE_CLOSE(quaternion_error, 0.0, 1e-5);

    BOOST_REQUIRE_EQUAL(base.angular_velocity(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.position(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.velocity(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.acceleration(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.time_sec(), 0.0);
    BOOST_REQUIRE_EQUAL(base.time_usec(), 0);
}

BOOST_AUTO_TEST_CASE(ZeroKalmanStateTest) {
    KalmanState base = KalmanState::zero(0);

    double quaternion_error =
        diff(base.attitude().unit_quaternion(), Quaterniond::Identity());
    BOOST_REQUIRE_CLOSE(quaternion_error, 0.0, 1e-5);

    BOOST_REQUIRE_EQUAL(base.angular_velocity(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.position(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.velocity(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.acceleration(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.time_sec(), 0.0);
    BOOST_REQUIRE_EQUAL(base.time_usec(), 0);

    BOOST_REQUIRE_EQUAL(base.gyro_bias(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.accel_bias(), Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(base.gravity_vector(), Vector3d(0, 0, -9.80665));
    BOOST_REQUIRE_EQUAL(base.magnetic_field_vector(), Vector3d(1, 0, 0));
}

BOOST_AUTO_TEST_CASE(StateModificationTest) {
    KalmanState base = KalmanState::zero(0);

    Vector3d vec(1, 2, 3);
    base.position() = vec;
    BOOST_REQUIRE_EQUAL(base.position(), vec);
}
