#include "gcs/XboxController.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

using namespace std;

namespace maav
{
namespace gcs
{
void XboxController::updateControllerState()
{
	// Update gamepad state
	GamepadUpdate();  // sets current gamepad state

	// Looks like GamepadStickNormXY doesn't work right,
	// so apply this workaround
	double leftAngle = GamepadStickAngle(controller, STICK_LEFT);
	double leftLength = GamepadStickLength(controller, STICK_LEFT);
	double rightAngle = GamepadStickAngle(controller, STICK_RIGHT);
	double rightLength = GamepadStickLength(controller, STICK_RIGHT);

	lx = cos(leftAngle) * leftLength;
	ly = sin(leftAngle) * leftLength;
	rx = cos(rightAngle) * rightLength;
	ry = sin(rightAngle) * rightLength;
}

void XboxController::setStickOutputRange(const FourElemArray& stick_ranges, double min_throttle)
{
	roll_range = stick_ranges[0];
	pitch_range = stick_ranges[1];
	yaw_rate_range = stick_ranges[2];
	thrust_range = stick_ranges[3];

	throttle_floor = min_throttle;
}

dji_t XboxController::getDesiredRpyt() const
{
	if (!GamepadIsConnected(controller))
	{
		cerr << "Controller not connected!" << endl;
	}

	dji_t msg;
	msg.utime = chrono::time_point_cast<chrono::microseconds>(chrono::system_clock::now())
					.time_since_epoch()
					.count();
	msg.roll = rx * roll_range + roll_range;
	msg.pitch = -ry * pitch_range + pitch_range;
	msg.yawRate = lx * yaw_rate_range + yaw_rate_range;
	msg.thrust = (thrust_range + throttle_floor) + ly * thrust_range;

	return msg;
}

const XboxController::FourElemArray& XboxController::getRawStickVals() const { return stick_vals; }
}  // namespace gcs

}  // namespace maav
