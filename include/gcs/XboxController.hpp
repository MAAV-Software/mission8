#ifndef MAAV_GCS_XBOX_CONTROLLER_HPP
#define MAAV_GCS_XBOX_CONTROLLER_HPP

#include "common/messages/dji_t.hpp"

#include <array>
#include "common/utils/xbox-controller/gamepad.h"

namespace maav
{
namespace gcs
{
class XboxController
{
	using FourElemArray = std::array<double, 4>;

   public:
	/**
	 * @brief Constructs an Xbox360 gamepad controller.
	 */
	XboxController()
	{
		GamepadInit();
		lx = 0;
		ly = 0;
		rx = 0;
		ry = 0;
	}

	/**
	 * @brief Constructs an Xbox360 gamepad controller with the given
	 * 		stick ranges and the given minimum throttle.
	 * @detail See setStickOutputRange for argument documentation.
	 */
	XboxController(const FourElemArray& stick_ranges, double min_throttle) : XboxController{}
	{
		setStickOutputRange(stick_ranges, min_throttle);
	}

	~XboxController() { GamepadShutdown(); }
	/**
	 * @brief Update the controller's current stick values.
	 */
	void updateControllerState();

	/**
	 * @brief Alter the range of outputs produced by each stick.
	 * @param stick_ranges An array whose elements correspond to the maximum
	 * 		absolute stick value that that the controller should report.
	 *
	 * 		Negative stick ranges will produce a runtime exception.
	 *
	 * @param min_throttle The thrust value that will be reported when the left
	 * 			stick is pushed all the way down.
	 * @detail The ordering of values goes: roll, pitch, yaw, and thrust.
	 *
	 * 		Example Values and Effects:
	 * 		- An Eigen::Vector4d of {1.0, 2.5, 3.0, and 0.4}, and a
	 * 		- min_throttle of 0
	 *
	 * 		Would allow getDesiredRpyt to produce dji_t's whose field values
	 * 		fall in the following ranges:
	 *
	 *		dji_t {
	 *			utime: //N/A, populated by utime
	 *			roll:		[-1.0, 1.0] radians from level
	 *			pitch:		[-2.5, 2.5] radians from level
	 *			yawRate:	[-3.0, 3.0] radians/sec from rest
	 *			thrust:		[0, 0.8] Newtons
	 *		}
	 */
	void setStickOutputRange(const FourElemArray& stick_ranges, double min_throttle = 0);

	/**
	 * @brief Get a dji_t message whose roll, pitch, yaw, and throttle
	 * 		values are populated directly with stick values and the current
	 * 		utime.
	 */
	dji_t getDesiredRpyt() const;

	/**
	 * @brief Get a four element array containing the current raw stick values,
	 * 		ordered as Roll, Pitch, Yaw, and Thrust.
	 *
	 * 		i.e. rx, ry, lx, ly
	 */
	const FourElemArray& getRawStickVals() const;

   private:
	// lx = left stick x values, positive values -> stick moved right
	// ly = left stick y values, positive values -> stick moved up
	// Range: [-1.0, 1.0]
	// etc.
	FourElemArray stick_vals;
	double& lx = stick_vals.at(2);
	double& ly = stick_vals.at(3);
	double& rx = stick_vals.at(0);
	double& ry = stick_vals.at(1);

	// Take stick values and divide by these values to get
	// values that will be returned by getDesiredRpyt
	double roll_range{1};
	double pitch_range{1};
	double yaw_rate_range{1};
	double thrust_range{8};

	double throttle_floor{0};

	GAMEPAD_DEVICE controller{GAMEPAD_0};  //"player one" controller
};

}  // namespace gcs

}  // namespace maav

#endif
