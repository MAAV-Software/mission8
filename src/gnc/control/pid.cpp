#include <gnc/control/pid.hpp>

namespace maav
{
namespace gnc
{
namespace control
{
Pid::Pid() : _kp(0), _ki(0), _kd(0), _eint(0), _eprev(0) {}
Pid::Pid(double p, double i, double d) : _kp(p), _ki(i), _kd(d), _eint(0), _eprev(0) {}
void Pid::setGains(double p, double i, double d)
{
	// First we want to reset the stored error
	reset();

	// Now set the new gains
	_kp = p;
	_ki = i;
	_kd = d;
}

double Pid::run(double e, double edot)
{
	_eint += e;
	return (_kp * e) + (_ki * _eint) + (_kd * edot);
}

double Pid::runDiscrete(double e, double dt)
{
	// Calculate derivative
	double edotDiscrete = (e - _eprev) / dt;
	_eprev = e;

	// Use that in the regular function
	return run(e, edotDiscrete);
}

void Pid::reset()
{
	_eint = 0;
	_eprev = 0;
}
}
}
}
