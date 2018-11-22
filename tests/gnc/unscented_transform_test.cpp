#define BOOST_TEST_MODULE UnscentedTransformTest
/**
 * Tests for both State and Kalman State
 */

#include <cmath>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <boost/test/unit_test.hpp>
#include <sophus/so3.hpp>

#include <gnc/State.hpp>
#include <gnc/kalman/unscented_transform.hpp>
#include "test_helpers.hpp"

using namespace boost::unit_test;
using namespace Eigen;

using namespace maav::gnc::kalman;
using namespace maav::gnc;

State identity(const State& state) { return state; }
BOOST_AUTO_TEST_CASE(IdentityTransformTest)
{
    UnscentedTransform<State> id(identity, 0.1, 2, 0);

    State state = State::zero(0);

    State transformed_state = id(state);

    constexpr double tol = 1e-5;
    BOOST_CHECK_LE(std::abs(diff(state.attitude(), transformed_state.attitude())), tol);
    BOOST_CHECK_LE(std::abs(diff(state.position(), transformed_state.position())), tol);
    BOOST_CHECK_LE(std::abs(diff(state.velocity(), transformed_state.velocity())), tol);
}

BOOST_AUTO_TEST_CASE(YAMLRead)
{
    YAML::Node config = YAML::Load("alpha: 0.1\nbeta: 2.0\nkappa: 0.1");
    UnscentedTransform<State> id(config);
    id.set_transformation(identity);

    double w_m_0 = -97.9011;
    double w_c_0 = -94.9111;
    double w = 5.4945;

    constexpr double tol = 1e-5;

    BOOST_CHECK_LE(std::abs(w_m_0 - id.m_weights()[0]), tol);
    BOOST_CHECK_LE(std::abs(w_c_0 - id.c_weights()[0]), tol);
    BOOST_CHECK_LE(std::abs(w - id.m_weights()[1]), tol);
    BOOST_CHECK_LE(std::abs(w - id.c_weights()[1]), tol);
}

void compare_states(const State& state, const State::ErrorStateVector& error_state)
{
    Sophus::SO3d e_attitude = Sophus::SO3d::exp(error_state.segment<3>(0));
    Eigen::Vector3d e_position = error_state.segment<3>(3);
    Eigen::Vector3d e_velocity = error_state.segment<3>(6);

    constexpr double tol = 1e-4;
    BOOST_CHECK_LE(diff(state.attitude(), e_attitude), tol);
    BOOST_CHECK_LE(diff(state.position(), e_position), tol);
    BOOST_CHECK_LE(diff(state.velocity(), e_velocity), tol);
}

BOOST_AUTO_TEST_CASE(SigmaPointsTest)
{
    using UT = UnscentedTransform<State>;

    UT id(identity, 0.25, 2, 0);

    State state = State::zero(0);

    State::CovarianceMatrix Sigma;
    // clang-format off
	Sigma << 
		1.9371,    2.0888,    1.4652,    1.2174,    1.5518,    1.2272,    1.6696,    1.2636,    1.6087,
    	2.0888,    2.8621,    1.9755,    1.6332,    2.1316,    1.5430,    2.3155,    1.6608,    2.4866,
    	1.4652,    1.9755,    1.8333,    1.0937,    1.7912,    1.0241,    1.4011,    0.9566,    1.9678,
    	1.2174,    1.6332,    1.0937,    1.1731,    1.1899,    0.8209,    1.1815,    0.9951,    1.6100,
    	1.5518,    2.1316,    1.7912,    1.1899,    2.2638,    1.2084,    1.9926,    0.9625,    2.2366,
    	1.2272,    1.5430,    1.0241,    0.8209,    1.2084,    1.0446,    1.4966,    0.9215,    1.2648,
    	1.6696,    2.3155,    1.4011,    1.1815,    1.9926,    1.4966,    2.6750,    1.2492,    2.1626,
    	1.2636,    1.6608,    0.9566,    0.9951,    0.9625,    0.9215,    1.2492,    1.3088,    1.2292,
    	1.6087,    2.4866,    1.9678,    1.6100,    2.2366,    1.2648,    2.1626,    1.2292,    2.9463;
    // clang-format on
    state.covariance() = Sigma;

    id(state);

    const UT::SigmaPoints& sig_pts = id.last_sigma_points();
    const UT::TransformedPoints& trans_pts = id.last_transformed_points();

    // clang-format off
	/**
	 * Values from matlab
	 */
	Eigen::Matrix<double, 9, 19> sigma_points;
	sigma_points << 
         0,    1.0438,   -1.0438,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
         0,    1.1256,   -1.1256,    0.5856,   -0.5856,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
         0,    0.7896,   -0.7896,    0.3799,   -0.3799,    0.5133,   -0.5133,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
         0,    0.6560,   -0.6560,    0.3078,   -0.3078,   -0.0384,    0.0384,    0.3651,   -0.3651,         0,         0,         0,         0,         0,         0,         0,         0,         0,         0,
         0,    0.8362,   -0.8362,    0.4402,   -0.4402,    0.3508,   -0.3508,   -0.0035,    0.0035,    0.5072,   -0.5072,         0,         0,         0,         0,         0,         0,         0,         0,
         0,    0.6613,   -0.6613,    0.2110,   -0.2110,   -0.0511,    0.0511,   -0.1068,    0.1068,    0.1014,   -0.1014,    0.2854,   -0.2854,         0,         0,         0,         0,         0,         0,
         0,    0.8997,   -0.8997,    0.4948,   -0.4948,   -0.2147,    0.2147,   -0.2360,    0.2360,    0.4440,   -0.4440,    0.2147,   -0.2147,    0.3246,   -0.3246,         0,         0,         0,         0,
         0,    0.6809,   -0.6809,    0.2865,   -0.2865,   -0.2111,    0.2111,    0.0459,   -0.0459,   -0.1574,    0.1574,    0.0619,   -0.0619,   -0.0911,    0.0911,    0.3269,   -0.3269,         0,         0,
         0,    0.8669,   -0.8669,    0.7222,   -0.7222,    0.2884,   -0.2884,    0.3443,   -0.3443,    0.2273,   -0.2273,    0.0499,   -0.0499,    0.3411,   -0.3411,    0.0095,   -0.0095,    0.1089,   -0.1089;
    // clang-format on

    constexpr double tol = 1e-5;
    for (size_t i = 0; i < sig_pts.size(); i++)
    {
        BOOST_REQUIRE_LE(diff(sig_pts[i].attitude(), trans_pts[i].attitude()), tol);
        BOOST_REQUIRE_LE(diff(sig_pts[i].position(), trans_pts[i].position()), tol);
        BOOST_REQUIRE_LE(diff(sig_pts[i].velocity(), trans_pts[i].velocity()), tol);
        compare_states(sig_pts[i], sigma_points.col(i));
    }
}
// TODO: Add more complex transformations
