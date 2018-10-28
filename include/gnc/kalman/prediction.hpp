#pragma once

#include <functional>

#include <yaml-cpp/yaml.h>

#include <gnc/kalman/history.hpp>
#include <gnc/kalman/unscented_transform.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
class UkfPrediction
{
   public:
	UkfPrediction(YAML::Node config);

	void operator()(const History::ConstIterator prev, const History::Iterator next);

   private:
	History::ConstIterator _prev;
	History::Iterator _next;

	KalmanState predict(const KalmanState& state);

	using PredictionUT = UnscentedTransform<KalmanState>;

	PredictionUT transformation;
	Eigen::Matrix<double, KalmanState::DoF, KalmanState::DoF> Q;

   protected:
	// For testing
	const PredictionUT& getUT() const { return transformation; }
};
}
}
}
