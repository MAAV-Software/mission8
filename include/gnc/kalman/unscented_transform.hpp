#pragma once

#include <algorithm>
#include <functional>
#include <vector>

#include <Eigen/Cholesky>

#include <gnc/kalman/kalman_state.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
template <class TargetSpace>
class UnscentedTransform
{
   public:
	using Transform = std::function<TargetSpace(const KalmanState&)>;

	UnscentedTransform(Transform transform, double alpha, double beta, double kappa)
		: _transformation(transform), _alpha(alpha), _beta(beta), _kappa(kappa)
	{
		constexpr size_t n = 1 + 2 * KalmanState::DIM;
		constexpr auto n_d = static_cast<double>(n);
		_lambda = _alpha * _alpha * (n_d + _kappa) - n_d;
		const double w_m_0 = _lambda / (n_d + _lambda);
		const double w_c_0 = w_m_0 + (1 - (_alpha * _alpha) + _beta);
		const double w = 1 / (2 * (n_d + _lambda));
		m_weights = std::vector<double>(n, w);
		c_weights = std::vector<double>(n, w);
		m_weights[0] = w_m_0;
		c_weights[0] = w_c_0;
	}

	/**
	 * Transforms the gaussian from the state space to a gaussian in some target
	 * space
	 */
	TargetSpace operator()(const KalmanState& state);

   private:
	using SigmaPoints = std::vector<KalmanState>;

	SigmaPoints generate_sigma_points(const KalmanState& state);

   private:
	Transform _transformation;

	double _lambda;

	double _alpha;

	double _beta;

	double _kappa;

	std::vector<double> m_weights;
	std::vector<double> c_weights;
};

template <class TargetSpace>
TargetSpace UnscentedTransform<TargetSpace>::operator()(const KalmanState& state)
{
	SigmaPoints sigma_points = generate_sigma_points(state);
	std::vector<TargetSpace> transformed_points;
	transformed_points.reserve(sigma_points.size());
	// Pass all sigma points through the transform
	std::transform(sigma_points.begin(), sigma_points.end(), std::back_inserter(transformed_points),
				   _transformation);

	// Recompute the mean and covariance
	TargetSpace result = TargetSpace::compute_gaussian(transformed_points, m_weights, c_weights);
	return result;
}

template <class TargetSpace>
typename UnscentedTransform<TargetSpace>::SigmaPoints
UnscentedTransform<TargetSpace>::generate_sigma_points(const KalmanState& state)
{
	constexpr size_t N = KalmanState::E_DIM;

	Eigen::LLT<KalmanState::CovarianceMatrix> decomp((static_cast<double>(N) + _lambda) *
													 state.covariance());
	const KalmanState::CovarianceMatrix L = decomp.matrixL();
	SigmaPoints points;
	points.reserve(1 + 2 * N);
	points.push_back(state);
	for (size_t i = 0; i < N; i++)
	{
		const KalmanState::ErrorStateVector& sigma_point_offset = L.col(i);

		KalmanState additive_point = state;
		KalmanState subtractive_point = state;

		additive_point += sigma_point_offset;
		subtractive_point += (-1 * sigma_point_offset);

		points.push_back(additive_point);
		points.push_back(subtractive_point);
	}

	return points;
}

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
