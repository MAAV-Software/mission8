#define BOOST_TEST_MODULE StateTest
/**
 * Tests for both BaseState and Kalman State
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <gnc/kalman/kalman_state.hpp>
#include <gnc/state/base_state.hpp>
#include "test_helpers.hpp"

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::state;
using namespace maav::gnc::kalman;

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

BOOST_AUTO_TEST_CASE(StateSimpleMeanTest) {
    size_t num_states = 2;
    std::vector<KalmanState> states(num_states, KalmanState::zero(200));
    std::vector<double> weights(num_states,
                                1.0 / static_cast<double>(num_states));
    KalmanState mean = KalmanState::mean(states, weights);

    BOOST_REQUIRE(mean.attitude().matrix() == Eigen::Matrix3d::Identity());
    BOOST_REQUIRE_EQUAL(mean.angular_velocity(), Eigen::Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(mean.position(), Eigen::Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(mean.velocity(), Eigen::Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(mean.acceleration(), Eigen::Vector3d::Zero());
    BOOST_REQUIRE_EQUAL(mean.time_usec(), 200);
}

BOOST_AUTO_TEST_CASE(StateQuaternionMeanTest) {
    size_t num_states = 2;
    std::vector<KalmanState> states(num_states, KalmanState::zero(0));
    std::vector<double> weights(num_states,
                                1.0 / static_cast<double>(num_states));
    KalmanState mean = KalmanState::mean(states, weights);

    constexpr double tol = 1e-5;
    BOOST_REQUIRE(diff(mean.attitude().unit_quaternion(),
                       Eigen::Quaterniond::Identity()) < tol);

    states[0].attitude() = Sophus::SO3d(Eigen::Quaterniond::Identity());
    states[1].attitude() =
        Sophus::SO3d(Eigen::Quaterniond(0.707106781, 0.707106781, 0, 0));

    mean = KalmanState::mean(states, weights);
    BOOST_REQUIRE(diff(mean.attitude().unit_quaternion(),
                       Eigen::Quaterniond(0.923879533, 0.382683432, 0, 0)) <
                  tol);

    states[0].attitude() = Sophus::SO3d(Eigen::Quaterniond::Identity());
    states[1].attitude() =
        Sophus::SO3d(Eigen::Quaterniond(0.707106781, 0, 0, 0.707106781));

    mean = KalmanState::mean(states, weights);
    BOOST_REQUIRE(diff(mean.attitude().unit_quaternion(),
                       Eigen::Quaterniond(0.923879533, 0, 0, 0.382683432)) <
                  tol);
}

BOOST_AUTO_TEST_CASE(StateWieghtedMeanTest) {
    size_t num_states = 2;
    std::vector<KalmanState> states(num_states, KalmanState::zero(0));
    std::vector<double> weights(num_states,
                                1.0 / static_cast<double>(num_states));
    constexpr double tol = 1e-5;

    states[0].position() = {1, 1, 1};
    states[1].position() = {2, 2, 2};

    KalmanState mean = KalmanState::mean(states, weights);

    Eigen::Vector3d diff = mean.position() - Eigen::Vector3d(1.5, 1.5, 1.5);
    BOOST_REQUIRE_LE(diff.norm(), tol);

    states[0].position() = {1, 1, 1};
    states[1].position() = {2, 2, 2};
    weights[0] = 0.3;
    weights[1] = 0.7;

    mean = KalmanState::mean(states, weights);

    diff = mean.position() - Eigen::Vector3d(1.7, 1.7, 1.7);
    BOOST_REQUIRE_LE(diff.norm(), tol);

    states.push_back(KalmanState::zero(0));
    weights.push_back(0.1);
    weights[1] = 0.6;

    mean = KalmanState::mean(states, weights);
    diff = mean.position() -
           Eigen::Vector3d(0.3 + 0.6 * 2, 0.3 + 0.6 * 2, 0.3 + 0.6 * 2);
    BOOST_REQUIRE_LE(diff.norm(), tol);
}

// TODO: add covariance tests
