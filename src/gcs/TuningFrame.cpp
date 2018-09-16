#include "gcs/TuningFrame.hpp"

#include "common/utils/debug.hpp"
#include <iostream>

#include <algorithm>
#include <functional>
#include <iterator>
#include <chrono>

using namespace std;
using namespace std::literals::string_literals;
using namespace std::chrono;

namespace maav
{
namespace gcs
{

//the number of grid columns that each PID control takes up
//row labels all take up only one grid column, so this is essentially a multiple
//of the size of those
constexpr auto SCALE_WIDTH = 4;

AutoRescaleSlider::AutoRescaleSlider()
{
	update_scale();
}

void AutoRescaleSlider::update_scale()
{
	set_range(0.0, 11.0*cur_scale);
	set_digits(max(cur_digits, 0));
}

bool AutoRescaleSlider::on_button_release_event(GdkEventButton* evt)
{
	Scale::on_button_release_event(evt);
	const auto value = get_value();

	//scale it down if the value is < cur_scale
	if (value < cur_scale) {
		cur_scale /= 10.0;
		cur_digits += 1;
		update_scale();
	}

	//scale it up if the value is the old maximum value
	else if (value == 11.0*cur_scale) {
		cur_scale *= 10.0;
		cur_digits -= 1;
		update_scale();
	}
	return false;
}

void AutoRescaleSlider::set_value(double val)
{
	while (val != 0.0 and val < cur_scale) {
		cur_scale /= 10.0;
		cur_digits += 1;
	}
	while (val >= 11.0*cur_scale) {
		cur_scale *= 10.0;
		cur_digits -= 1;
	}
	update_scale();
	Scale::set_value(val);
}

TuningFrame::PIDScale::PIDScale(const char* name, const GCSConsts& consts_in)
	: label{"<tt>"s + name + "</tt>", Gtk::ALIGN_START, Gtk::ALIGN_END},
	CONSTS{consts_in}
{
	set_spacing(CONSTS.SMALL_SPACE);

	label.set_use_markup();

	add(label);
	pack_start(scale);
}

double TuningFrame::PIDScale::get_value() const
{
	return scale.get_value();
}

void TuningFrame::PIDScale::set_value(double v)
{
	scale.set_value(v);
}

TuningFrame::PIDControl::PIDControl(const GCSConsts& consts_in)
	: Gtk::Box{Gtk::ORIENTATION_VERTICAL}, CONSTS{consts_in}
{
	add(p), add(i), add(d);
}

pid_gains_t TuningFrame::PIDControl::get_value() const
{
	pid_gains_t pid;
	pid.p = p.get_value();
	pid.i = i.get_value();
	pid.d = d.get_value();
	return pid;
}

void TuningFrame::PIDControl::set_value(const pid_gains_t& v)
{
	p.set_value(v.p), i.set_value(v.i), d.set_value(v.d);
}

TuningFrame::TuningFrame(GlibZCM& zcm, const GCSConsts& consts_in)
	: Gtk::Frame("Tuning"), CONSTS{consts_in},
	controls{PIDControl{consts_in}, PIDControl{consts_in}, PIDControl{consts_in},
		PIDControl{consts_in}, PIDControl{consts_in}, PIDControl{consts_in},
		PIDControl{consts_in}},
	ctrl_params{zcm, CTRL_PARAMS_CHANNEL, "PARAMS_NULL", get_ctrl_params()},
	saved_params{load_params()}
{
	MAAV_DEBUG("Initializing Tuning Frame");
	set_label_align(Gtk::ALIGN_CENTER);

	grid.set_column_homogeneous(true);
	grid.set_row_spacing(CONSTS.MED_SPACE);
	grid.set_column_spacing(CONSTS.MED_SPACE);
	grid.set_border_width(CONSTS.MED_SPACE);

	grid.attach(value_label, 1, 0, SCALE_WIDTH, 1);
	grid.attach(rate_label, 1 + SCALE_WIDTH, 0, SCALE_WIDTH, 1);

	constexpr const char* ROW_NAMES[] = {"X", "Y", "Z", "Yaw"};
	for(unsigned row = 0; row < 4; ++row) {
		row_labels[row].set_label(ROW_NAMES[row]);
		grid.attach(row_labels[row], 0, 1 + 2*row, 1, 1);
		grid.attach(controls[2*row], 1, 1 + 2*row, SCALE_WIDTH, 1);

		//the second (rate) PID should not be added for the Yaw row
		if (row != 3) {
			grid.attach(separators[row], 0, 2 + 2*row, 1 + 2*SCALE_WIDTH, 1);
			grid.attach(controls[2*row + 1], 1 + SCALE_WIDTH, 1 + 2*row,
				SCALE_WIDTH, 1);
		}
	}

	set_ctrl_params(saved_params);

	//the control parameters should be sent out via cmd() if the publish button is
	//pressed
	publish.signal_clicked().connect([this]() {
		MAAV_DEBUG("Publishing gains");
		ctrl_params.cmd(get_ctrl_params());
	});

	//if reset is pressed, they should be reset
	//update_buttons isn't called automatically here, so it is called manually
	reset.signal_clicked().connect([this]() {
		MAAV_DEBUG("Resetting gains to last published");
		set_ctrl_params(ctrl_params.stat());
		update_buttons(ctrl_params.stat());
	});

	//the UI tracks control parameter updates as long as the UI values match the
	//old control parameters; if they've been updated by the user, they will no
	//longer track updates
	ctrl_params.signal_update().connect([this](const ctrl_params_t& stat) {
		if (ctrl_params.stat() == get_ctrl_params()) set_ctrl_params(stat);
		update_buttons(stat);
	});

	save.signal_clicked().connect([this](){
		vector<pid_gains_t> pid_gains;
		array<double, 21> gains;
		ctrl_params_t params = get_ctrl_params();
		pid_gains.insert(pid_gains.end(), begin(params.value), end(params.value));
		pid_gains.insert(pid_gains.end(), begin(params.rate), end(params.rate));
		for(unsigned i = 0, j = 0; i < pid_gains.size(); ++i, j+=3) {
			gains[j] = pid_gains[i].p;
			gains[j + 1] = pid_gains[i].i;
			gains[j + 2] = pid_gains[i].d;
		}
		CONSTS.writeGains(gains);
	});

	//update_buttons should also be called whenever any of the slider's values
	//changes
	for (auto& control : controls) {
		control.when_value_changed([this]() {
			update_buttons(ctrl_params.stat());
		});
	}

	btns.set_layout(Gtk::BUTTONBOX_EXPAND);
	btns.set_spacing(CONSTS.MED_SPACE);
	btns.pack_start(save), btns.pack_start(reset), btns.pack_start(publish);
	grid.attach(btns, 0, 8, 1 + 2*SCALE_WIDTH, 1);

	add(grid);
	show_all_children();
}

ctrl_params_t TuningFrame::get_ctrl_params() const
{
	ctrl_params_t params;
	params.utime = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
	for(unsigned i = 0, j = 0; i < 4; ++i) {
		params.value[i] = controls[j++].get_value();
		// rate has no yaw gain
		if(i != 3) {
			params.rate[i] = controls[j++].get_value();
		}
	}
	return params;
}

void TuningFrame::set_ctrl_params(const ctrl_params_t& params)
{
	for (int i{0}; i < 7; ++i) {
		if (i < 4) controls[i].set_value(params.value[i]);
		else controls[i].set_value(params.rate[i - 4]);
	}
}

void TuningFrame::update_buttons(const ctrl_params_t& stat)
{
	//the buttons both enabled if the UI values have changed from the status;
	//otherwise they're both disabled
	const auto sensitive = stat != get_ctrl_params();
	reset.set_sensitive(sensitive);
	publish.set_sensitive(sensitive);
}

ctrl_params_t TuningFrame::load_params()
{
	ctrl_params_t params;
	pid_gains_t gains;
	gains.p = CONSTS.POS_X[0];
	gains.i = CONSTS.POS_X[1];
	gains.d = CONSTS.POS_X[2];
	params.value[0] = gains;
	gains.p = CONSTS.POS_Y[0];
	gains.i = CONSTS.POS_Y[1];
	gains.d = CONSTS.POS_Y[2];
	params.value[1] = gains;
	gains.p = CONSTS.POS_Z[0];
	gains.i = CONSTS.POS_Z[1];
	gains.d = CONSTS.POS_Z[2];
	params.value[2] = gains;
	gains.p = CONSTS.POS_YAW[0];
	gains.i = CONSTS.POS_YAW[1];
	gains.d = CONSTS.POS_YAW[2];
	params.value[3] = gains;
	gains.p = CONSTS.RATE_X[0];
	gains.i = CONSTS.RATE_X[1];
	gains.d = CONSTS.RATE_X[2];
	params.rate[0] = gains;
	gains.p = CONSTS.RATE_Y[0];
	gains.i = CONSTS.RATE_Y[1];
	gains.d = CONSTS.RATE_Y[2];
	params.rate[1] = gains;
	gains.p = CONSTS.RATE_Z[0];
	gains.i = CONSTS.RATE_Z[1];
	gains.d = CONSTS.RATE_Z[2];
	params.rate[2] = gains;
	saved_params = params;
	return params;
}

}
}
