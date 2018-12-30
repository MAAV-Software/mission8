#ifndef LOWPASS_HPP_
#define LOWPASS_HPP_

namespace maav
{
class LowPass
{
public:
    LowPass();
    LowPass(double alpha, double state);
    LowPass(double alpha);

    void run(double input);
    double getState() const;

private:
    double alpha_;
    double state_;
};
}

#endif /* LowPass.hpp */