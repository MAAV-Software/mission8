#ifndef MAAV_GCS_TUNING_FRAME_HPP
#define MAAV_GCS_TUNING_FRAME_HPP

#include "GCSConsts.hpp"
#include "StatCmd.hpp"
#include "zcm_type_ops.hpp"

#include <gtkmm/box.h>
#include <gtkmm/button.h>
#include <gtkmm/buttonbox.h>
#include <gtkmm/frame.h>
#include <gtkmm/grid.h>
#include <gtkmm/label.h>
#include <gtkmm/scale.h>
#include <gtkmm/separator.h>

#include <array>
#include <fstream>

namespace maav
{
namespace gcs
{
/**
 * @brief An auto-rescaling form of Gtk::Scale
 *
 * @details This control implements a slider which automatically changes its
 * scale by a factor of 10 when the thumb is released on a low (non-zero) value
 * or on the highest value, which makes it easier to enter very small or very
 * large amounts
 */
class AutoRescaleSlider : public Gtk::Scale
{
	// the current scale value; the range of the slider will be from 0 to
	// 11*cur_scale
	double cur_scale{1.0};

	// the current number of digits to display; setting this to 2 initially gives
	// about 3 digits of accuracy
	int cur_digits{2};

	// updates the scale to reflect new values of cur_scale and cur_digits
	void update_scale();

   protected:
	// handle button release events to adjust the range when the user lets go of
	// the thumb
	bool on_button_release_event(GdkEventButton*) override;

   public:
	/**
	 * @brief Creates a default AutoRescaleSlider with an initial range from 0
	 * to 11
	 */
	AutoRescaleSlider();

	/**
	 * @brief Sets the value of this slider, automatically rescaling it as needed
	 */
	void set_value(double);
};

/**
 * @brief The GCS tuning frame
 *
 * @details This frame contains a bunch of sliders for specifying PID values for
 * tuning purposes and a button to send those new values to the vehicle
 */
class TuningFrame : public Gtk::Frame
{
	// a single slider as used in PIDControl below
	class PIDScale : public Gtk::Box
	{
		AutoRescaleSlider scale;
		Gtk::Label label;
		const GCSConsts CONSTS;

	   public:
		// label text is specified via the constructor
		PIDScale(const char* name, const GCSConsts& consts_in);

		// setters and getters for the scale value, which essentially just forward to
		// the scale
		double get_value() const;
		void set_value(double);

		// registers f to be called whenever the slider value is updated
		template <typename F>
		void when_value_changed(F f)
		{
			scale.signal_value_changed().connect(f);
		}
	};

	// a sub-control for collecting PID values
	class PIDControl : public Gtk::Box
	{
		const GCSConsts& CONSTS;
		PIDScale p{"P", CONSTS}, i{"I", CONSTS}, d{"D", CONSTS};

	   public:
		explicit PIDControl(const GCSConsts& consts_in);

		// getters and setters which update/read the three sub-controls
		pid_gains_t get_value() const;
		void set_value(const pid_gains_t&);

		// registers f to be called whenever the value of any of the three sliders
		// that this contains is updated
		template <typename F>
		void when_value_changed(F f)
		{
			p.when_value_changed(f);
			i.when_value_changed(f);
			d.when_value_changed(f);
		}
	};

	// Constants container
	const GCSConsts& CONSTS;

	// the two column labels
	Gtk::Label value_label{"Value"}, rate_label{"Rate"};

	// the four row labels (which will be filled in later)
	std::array<Gtk::Label, 4> row_labels;

	// the three separators for dividing the rows
	std::array<Gtk::Separator, 3> separators;

	// all of the PID controls
	std::array<PIDControl, 7> controls;

	// the grid that all of this stuff goes into
	Gtk::Grid grid;

	// the buttons for publishing and resetting all of those values, and their
	// button box
	Gtk::ButtonBox btns{Gtk::ORIENTATION_HORIZONTAL};
	Gtk::Button reset{"Reset Gains"};
	Gtk::Button publish{"Publish Gains"};
	Gtk::Button save{"Save Gains"};

	// the StatCmd for coordinating updates with the controller running on the
	// vehicle
	StatCmd<ctrl_params_t> ctrl_params;

	// getters and setters for all of the control parameters as a unit
	ctrl_params_t get_ctrl_params() const;
	void set_ctrl_params(const ctrl_params_t&);

	// updates the two buttons to reflect a new control parameter status
	void update_buttons(const ctrl_params_t&);

	// loads gains from yaml config
	ctrl_params_t load_params();

	// The current saved params
	ctrl_params_t saved_params;

   public:
	/**
	 * @brief Sets up a tuning frame
	 */
	TuningFrame(GlibZCM&, const GCSConsts&);
};
}
}

#endif
