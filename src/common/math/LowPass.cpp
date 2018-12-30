#include <common/math/LowPass.hpp>

namespace maav
{
LowPass::LowPass() : alpha_(0), state_(0) {}
LowPass::LowPass(double alpha, double state) : alpha_(alpha), state_(state) {}
LowPass::LowPass(double alpha) : alpha_(alpha), state_(0) {}
void LowPass::run(double input) { state_ = (alpha_ * input) + ((1.0f - alpha_) * state_); }
double LowPass::getState() const { return state_; }
}