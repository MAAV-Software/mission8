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

BOOST_AUTO_TEST_CASE(ZeroBaseStateTest)
{
	BaseState base = BaseState::zero(0);

	double quaternion_error = diff(base.attitude().unit_quaternion(), Quaterniond::Identity());
	BOOST_REQUIRE_CLOSE(quaternion_error, 0.0, 1e-5);

	BOOST_REQUIRE_EQUAL(base.angular_velocity(), Vector3d::Zero());
	BOOST_REQUIRE_EQUAL(base.position(), Vector3d::Zero());
	BOOST_REQUIRE_EQUAL(base.velocity(), Vector3d::Zero());
	BOOST_REQUIRE_EQUAL(base.acceleration(), Vector3d::Zero());
	BOOST_REQUIRE_EQUAL(base.time_sec(), 0.0);
	BOOST_REQUIRE_EQUAL(base.time_usec(), 0);
}

BOOST_AUTO_TEST_CASE(ZeroKalmanStateTest)
{
	KalmanState base = KalmanState::zero(0);

	double quaternion_error = diff(base.attitude().unit_quaternion(), Quaterniond::Identity());
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

BOOST_AUTO_TEST_CASE(StateModificationTest)
{
	KalmanState base = KalmanState::zero(0);

	Vector3d vec(1, 2, 3);
	base.position() = vec;
	BOOST_REQUIRE_EQUAL(base.position(), vec);
}

BOOST_AUTO_TEST_CASE(UnweightedGaussianTest)
{
	std::array<KalmanState, KalmanState::N> states;
	states.fill(KalmanState::zero(200));
	std::array<double, KalmanState::N> weights;
	weights.fill(1.0 / static_cast<double>(KalmanState::N));

	Sophus::SO3d mean_rot(Eigen::Quaterniond(0.503, -0.002, -0.820, 0.273));
	mean_rot.normalize();
	Eigen::Vector3d mean_position(1.0849, 4.0597, -1.1371);
	Eigen::Vector3d mean_velocity(0.1570, 0.2582, 0.0304);

	/**
	 * Values computed in Matlab
	 */
	KalmanState::CovarianceMatrix output;
	output << 0.0305, 0.0344, 0.0221, 0.0313, 0.0275, 0.0229, 0.0246, 0.0328, 0.0372, 0.0344,
		0.0499, 0.0298, 0.0402, 0.0319, 0.0332, 0.0342, 0.0457, 0.0480, 0.0221, 0.0298, 0.0205,
		0.0240, 0.0201, 0.0174, 0.0221, 0.0292, 0.0325, 0.0313, 0.0402, 0.0240, 0.0432, 0.0346,
		0.0323, 0.0274, 0.0379, 0.0483, 0.0275, 0.0319, 0.0201, 0.0346, 0.0320, 0.0259, 0.0248,
		0.0332, 0.0413, 0.0229, 0.0332, 0.0174, 0.0323, 0.0259, 0.0291, 0.0240, 0.0332, 0.0338,
		0.0246, 0.0342, 0.0221, 0.0274, 0.0248, 0.0240, 0.0379, 0.0465, 0.0382, 0.0328, 0.0457,
		0.0292, 0.0379, 0.0332, 0.0332, 0.0465, 0.0597, 0.0506, 0.0372, 0.0480, 0.0325, 0.0483,
		0.0413, 0.0338, 0.0382, 0.0506, 0.0627;

	Eigen::Matrix<double, KalmanState::DoF, KalmanState::N> points;
	points << 0, 0.2050, -0.2525, 0.0964, 0.0356, -0.1462, -0.0485, 0.0383, 0.3999, 0.3095, -0.1509,
		0.3392, 0.0811, -0.0070, 0.0799, -0.0229, -0.0139, 0.1665, 0.0601, 0, 0.3555, -0.2217,
		-0.0005, 0.1039, -0.0173, -0.0103, 0.1353, 0.5113, 0.3183, -0.1421, 0.3081, 0.1699, -0.1105,
		-0.0065, -0.0980, -0.2789, 0.3145, 0.1931, 0, 0.1613, -0.0995, -0.0662, 0.0437, -0.0875,
		-0.0049, 0.0819, 0.2939, 0.2431, -0.1257, 0.2955, 0.1628, 0.0317, -0.0213, -0.0376, -0.1757,
		0.1074, 0.1115, 1.0000, 1.3739, 0.6971, 1.0811, 1.0510, 0.9614, 0.8959, 1.0834, 1.5077,
		1.4076, 0.9360, 1.3636, 1.0348, 0.9103, 0.9817, 1.0931, 0.8358, 1.2803, 1.1188, 4.0000,
		4.3053, 3.7101, 3.9903, 3.9410, 3.9719, 3.9361, 4.0353, 4.4793, 4.2883, 3.9205, 4.3691,
		3.9857, 3.9857, 4.1061, 3.9757, 3.9530, 4.1197, 4.0615, -1.2000, -0.8468, -1.4044, -1.0734,
		-1.1736, -1.1513, -1.2343, -1.1823, -0.7574, -1.0198, -1.2378, -1.0022, -1.1358, -1.3523,
		-1.1671, -1.2873, -1.3810, -0.8950, -1.1031, 0.1000, 0.2857, -0.1796, 0.0286, 0.2355,
		-0.0491, 0.1845, 0.1937, 0.5656, 0.2044, -0.0972, 0.3141, 0.4431, 0.1289, 0.2473, -0.1300,
		-0.0689, 0.2521, 0.3236, 0.2000, 0.4502, -0.0628, 0.1527, 0.3110, -0.0074, 0.1780, 0.2728,
		0.7762, 0.3406, -0.0856, 0.5306, 0.5803, 0.1564, 0.3604, -0.0808, -0.0794, 0.4500, 0.4631,
		-0.0500, 0.2814, -0.3700, -0.2115, -0.0258, -0.1618, -0.1478, 0.1252, 0.5336, 0.4227,
		-0.2342, 0.4737, 0.0722, -0.0310, -0.0420, -0.0283, -0.3021, 0.1213, 0.1516;

	states[0].attitude() = mean_rot;
	states[0].position() = {1, 4, -1.2};
	states[0].velocity() = {0.1, 0.2, -0.05};
	for (size_t i = 1; i < KalmanState::N; i++)
	{
		const KalmanState::ErrorStateVector& vec = points.block<KalmanState::DoF, 1>(0, i);
		states[i].attitude() = mean_rot * Sophus::SO3d::exp(vec.segment<3>(0));
		states[i].position() = vec.segment<3>(3);
		states[i].velocity() = vec.segment<3>(6);
	}

	constexpr double tol = 1e-3;

	KalmanState computed_gaussian = KalmanState::compute_gaussian(states, weights, weights);
	BOOST_CHECK_LE(diff(computed_gaussian.attitude().unit_quaternion(), mean_rot.unit_quaternion()),
				   tol);
	BOOST_CHECK_LE((computed_gaussian.position() - mean_position).norm(), tol);
	BOOST_CHECK_LE((computed_gaussian.velocity() - mean_velocity).norm(), tol);

	BOOST_CHECK_LE((computed_gaussian.covariance() - output).norm(), tol);
}
