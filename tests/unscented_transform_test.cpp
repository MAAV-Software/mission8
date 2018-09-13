#define BOOST_TEST_MODULE UnscentedTransformTest
/**
 * Tests for both BaseState and Kalman State
 */

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <gnc/kalman/kalman_state.hpp>
#include <gnc/kalman/unscented_transform.hpp>
#include "test_helpers.hpp"

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::state;
using namespace maav::gnc::kalman;

KalmanState identity(const KalmanState& state) { return state; }

BOOST_AUTO_TEST_CASE(IdentityTransformTest) {

    UnscentedTransform<KalmanState> id(identity, 0.1, 2, 0);

    KalmanState state = KalmanState::zero(0);

    KalmanState transformed_state = id(state);

    constexpr double tol = 1e-5;
    BOOST_REQUIRE(std::abs(diff(state.attitude(), transformed_state.attitude())) < tol);
    BOOST_REQUIRE(std::abs(diff(state.position(), transformed_state.position())) < tol);
    BOOST_REQUIRE(std::abs(diff(state.velocity(), transformed_state.velocity())) < tol);

}
