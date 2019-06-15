#include <utility>

#include <Eigen/Geometry>

#include "QualisysEstimator.hpp"

using Eigen::Vector3f;
using Eigen::Matrix3f;

namespace qualisys
{

QualisysEstimator::QualisysEstimator() : Rprev_(Matrix3f::Identity()), kf_(KalmanFilter()) {}

QualisysEstimator::QualisysEstimator(const KalmanFilter::StateCov& P, float max_accel, float lin_meas_noise)
	: Rprev_(Matrix3f::Identity()),
	  kf_(KalmanFilter(KalmanFilter::StateVec::Zero(), 
				       P, 
					   max_accel * KalmanFilter::StateCov::Identity(), 
					   lin_meas_noise * KalmanFilter::MeasCov::Identity())) {}

State QualisysEstimator::operator()(float dt, const Vector3f& pos, const Matrix3f& rot)
{
	State state; // default constructor needed (especially to zero-init ang_vel)
	
	// estimate position and velocity using linear kalman filter
	auto lin_est      = kf_(dt, pos);
	state.pos         = lin_est.first.topRows<3>();
	state.vel         = lin_est.first.bottomRows<3>();
	state.pos_cov     = lin_est.second.topLeftCorner<3, 3>();
	state.vel_cov     = lin_est.second.bottomRightCorner<3, 3>();
	state.pos_vel_cov = lin_est.second.topRightCorner<3, 3>();
	
	// estimate angular velocity using 1-step differentiation
	state.rot = rot; // pass through rot

	if (dt > 1e-6) // only differentiate if dt is not too small
	{
		auto Rdot  = (rot - Rprev_) / dt;
		
		/* Omega = Rdot * R^T = [   0, -w_z,  w_y;
		 *						  w_z,    0, -w_x;
		 *						 -w_y,  w_x,    0]
		 * where w is the angular velocity vector.
		 */
		auto Omega = Rdot * rot.transpose();

		// extract angular velocities
		state.ang_vel.x() = Omega(2, 1);
		state.ang_vel.y() = Omega(0, 2);
		state.ang_vel.z() = Omega(1, 0);
	}

	Rprev_ = rot; // update Rprev_

	return state;
}

}
