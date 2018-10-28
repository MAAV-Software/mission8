
#include <sophus/average.hpp>

#include <gnc/constants.hpp>
#include <gnc/kalman/kalman_state.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
KalmanState::KalmanState(uint64_t time_usec) : BaseState(time_usec) {}
KalmanState KalmanState::zero(uint64_t time_usec)
{
	KalmanState new_state(time_usec);
	new_state.attitude() = Sophus::SO3d(Eigen::Quaterniond::Identity());
	new_state.angular_velocity() = Eigen::Vector3d::Zero();
	new_state.position() = Eigen::Vector3d::Zero();
	new_state.velocity() = Eigen::Vector3d::Zero();
	new_state.acceleration() = Eigen::Vector3d::Zero();
	new_state.gyro_bias() = Eigen::Vector3d::Zero();
	new_state.accel_bias() = Eigen::Vector3d::Zero();
	new_state.gravity_vector() = {0.0, 0.0, -constants::STANDARD_GRAVITY};
	new_state.magnetic_field_vector() = {1.0, 0.0, 0.0};
	// TODO: Add starting covariance scalar
	new_state.covariance() = CovarianceMatrix::Identity();
	return new_state;
}

const Eigen::Vector3d& KalmanState::gyro_bias() const { return _gyro_bias; }
Eigen::Vector3d& KalmanState::gyro_bias() { return _gyro_bias; }
const Eigen::Vector3d& KalmanState::accel_bias() const { return _accel_bias; }
Eigen::Vector3d& KalmanState::accel_bias() { return _accel_bias; }
const Eigen::Vector3d& KalmanState::gravity_vector() const { return _gravity; }
Eigen::Vector3d& KalmanState::gravity_vector() { return _gravity; }
const Eigen::Vector3d& KalmanState::magnetic_field_vector() const { return _magnetic_field; }
Eigen::Vector3d& KalmanState::magnetic_field_vector() { return _magnetic_field; }
const KalmanState::CovarianceMatrix& KalmanState::covariance() const { return _covar; }
KalmanState::CovarianceMatrix& KalmanState::covariance() { return _covar; }
KalmanState KalmanState::mean(const std::array<KalmanState, N>& sigma_points,
							  const std::array<double, N>& weights)
{
	uint64_t time = sigma_points[0].time_usec();
	KalmanState mean_state(time);

	// Use only the previous mean's transformed point because this is linear with respect to the
	// attitude
	mean_state.attitude() = sigma_points[0].attitude();
	for (size_t i = 0; i < sigma_points.size(); i++)
	{
		mean_state.position() += sigma_points[i].position() * weights[i];
		mean_state.velocity() += sigma_points[i].velocity() * weights[i];

		// TODO: Determine if other values are necessary to average.
		// TODO: Use other estimated parameters
	}
	return mean_state;
}

KalmanState::CovarianceMatrix KalmanState::cov(const KalmanState& mean,
											   const std::array<KalmanState, N>& sigma_points,
											   const std::array<double, N>& weights)
{
	std::vector<ErrorStateVector> points;
	for (const KalmanState& state : sigma_points)
	{
		ErrorStateVector e_state;
		Eigen::Vector3d q_err = (mean.attitude().inverse() * state.attitude()).log();
		e_state.block<3, 1>(0, 0) = q_err;
		e_state.block<3, 1>(3, 0) = state.position();
		e_state.block<3, 1>(6, 0) = state.velocity();

		points.push_back(e_state);
		// TODO: add more states to estimate
	}

	ErrorStateVector mean_e_state;
	Eigen::Vector3d q_err = Eigen::Vector3d::Zero();
	mean_e_state.block<3, 1>(0, 0) = q_err;
	mean_e_state.block<3, 1>(3, 0) = mean.position();
	mean_e_state.block<3, 1>(6, 0) = mean.velocity();

	CovarianceMatrix new_covariance = CovarianceMatrix::Zero();
	for (size_t i = 0; i < sigma_points.size(); i++)
	{
		ErrorStateVector residual = points[i] - mean_e_state;
		new_covariance += weights[i] * residual * residual.transpose();
	}

	return new_covariance;
}

KalmanState KalmanState::compute_gaussian(const std::array<KalmanState, N>& points,
										  const std::array<double, N>& m_weights,
										  const std::array<double, N>& c_weights)
{
	KalmanState mu = KalmanState::mean(points, m_weights);
	mu.covariance() = KalmanState::cov(mu, points, c_weights);
	return mu;
}

KalmanState& KalmanState::operator+=(const KalmanState::ErrorStateVector& e_state)
{
	const Sophus::SO3d attitude_err = Sophus::SO3d::exp(e_state.head(3));
	const Eigen::Vector3d position_err = e_state.segment(3, 3);
	const Eigen::Vector3d velocity_err = e_state.segment(6, 3);

	attitude() *= attitude_err;
	position() += position_err;
	velocity() += velocity_err;

	return *this;
}

KalmanState::ErrorStateVector KalmanState::operator-(const KalmanState& other) const
{
	ErrorStateVector difference;
	difference.segment<3>(0) = (attitude() * other.attitude().inverse()).log();
	difference.segment<3>(3) = position() - other.position();
	difference.segment<3>(6) = velocity() - other.velocity();

	return difference;
}

}  // namespace kalman
}  // namespace gnc
}  // namespace maav
