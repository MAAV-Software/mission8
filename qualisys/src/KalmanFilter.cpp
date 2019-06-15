#include <functional>

#include <Eigen/LU>

#include "KalmanFilter.hpp"

using std::pair;
using std::make_pair;
using Eigen::Matrix;
using Eigen::Vector3f;

namespace qualisys
{

KalmanFilter::KalmanFilter()
	: x_(StateVec::Zero()), P_(0.01 * StateCov::Identity()), Q_(0.01 * StateCov::Identity()), 
	  R_(0.0001 * MeasCov::Identity()), A_(StateCov::Identity())
{
	// H = [I_3x3, 0_3x3];
	H_.setZero();
	H_(0, 0) = H_(1, 1) = H_(2, 2) = 1.0;
}

KalmanFilter::KalmanFilter(const StateVec& x, const StateCov& P, 
						   const StateCov& Q, const MeasCov& R)
	: x_(x), P_(P), Q_(Q), R_(R), A_(StateCov::Identity())
{
	// H = [I_3x3, 0_3x3];
	H_.setZero();
	H_(0, 0) = H_(1, 1) = H_(2, 2) = 1.0;
}

pair<KalmanFilter::StateVec, KalmanFilter::StateCov> KalmanFilter::operator()(float dt, const MeasVec& z)
{
	/* Set up dynamics (A) and covariance weights (W) using dt.
	 * Originally, A = I_6x6, now A = [I_3x3, dt * I_3x3;
	 *								   0_3x3,      I_3x3];
	 * Originally, Q = max_accel * I_6x6;
	 * Set W = block_diag([0.5 * dt^2 * I_3x3, dt * I_3x3]) to get actual 
	 * process covariance W * Q to model noise due to double integration of 
	 * acceleration.
	 */
	A_.topRightCorner<3, 3>() = Vector3f(dt, dt, dt).asDiagonal();
	StateCov W                = dt * StateCov::Identity();
	W.topLeftCorner<3, 3>()  *= dt / 2.0;
	
	// prediction
	x_ = A_ * x_;
	P_ = (A_ * P_ * A_.transpose()) + (W * Q_);
	
	// Kalman Gain
	using KalmanGain = Matrix<float, State::num_lin_states, State::num_lin_meas>;
	KalmanGain K = P_ * H_.transpose() * ((H_ * P_ * H_.transpose()) + R_).inverse();
	
	// correction
	x_ += K * (z - (H_ * x_));
	P_  = (StateCov::Identity() - (K * H_)) * P_;
	
	return make_pair(x_, P_);
}

}
