#include <Eigen/Core>

#include "State.hpp"

using Eigen::Vector3f;
using Eigen::Matrix3f;

namespace qualisys
{

State::State()
		: pos(Vector3f::Zero()), vel(Vector3f::Zero()),
		  rot(Matrix3f::Identity()), ang_vel(Vector3f::Zero()),
		  pos_cov(Matrix3f::Identity()), 
		  vel_cov(Matrix3f::Identity()),
		  pos_vel_cov(Matrix3f::Identity()) {}
}
